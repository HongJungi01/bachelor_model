/*
 * SemanticMaskStore.h
 *
 * Per-keyframe semantic point cache, anchored by RTAB-Map nodeId.
 *
 * Key design principle: ray-cast (depth read / ray-plane intersection) is
 * pose-independent — it operates in camera space and depends only on the
 * depth image and intrinsics.  It is therefore done ONCE when the labeled
 * boxes arrive (setLabeledBoxes) and the resulting camera-frame 3-D points
 * are stored, each tagged with the grid code derived from its YOLO label.
 *
 * When the sidecar returns a per-frame union mask alongside boxes, the
 * mask defines the wall pixels exactly (sharp). Box-only rasterisation is
 * kept as a fallback for mock mode and parse failures, but produces fat
 * axis-aligned rectangles that overshoot thin diagonal features like
 * parking lines — use the mask path whenever possible.
 *
 * applyTo() then only needs to apply the current corrected pose transform
 * (a cheap affine multiply) to project those points onto the map grid.
 * Loop-closure pose corrections flow through automatically because poses
 * are looked up fresh on every applyTo() call.
 *
 * Tag correction (latest observation wins): YOLO masks occasionally
 * overshoot the real feature, and a write-once overlay would keep those
 * wrong tags forever. Each keyframe therefore also stores sparse "clear"
 * samples — floor pixels it observed with valid depth that were NOT part
 * of any mask. A cell keeps its tag only while the NEWEST keyframe that
 * observed it saw it masked: one single mask-free observation of the
 * cell permanently ERASES the wall samples on it (no voting, no
 * threshold). Same-keyframe ties go to the mask so a frame's own clear
 * lattice cannot kill the cells it just tagged. A later genuine
 * re-detection re-tags the cell with fresh samples.
 *
 * Grid codes (semantic overlay on int8 occupancy grid):
 *   1      wall-like        centerLine, parkingLine
 *   0      skip             Arrow, rubberCone, sign, word, anything else
 */

#ifndef SEMANTICMASKSTORE_H_
#define SEMANTICMASKSTORE_H_

#include "SemanticBackproject.h"
#include "SemanticLabeledBox.h"

#include "rtabmap/core/Transform.h"
#include "rtabmap/core/CameraModel.h"

#include <opencv2/core.hpp>
#include <algorithm>
#include <unordered_map>
#include <map>
#include <mutex>
#include <cmath>
#include <string>
#include <vector>

class SemanticMaskStore
{
public:
    struct Entry {
        // Camera-frame 3-D points + parallel grid codes, computed once at
        // setLabeledBoxes() time. Pose-independent; only the transform in
        // applyTo() varies.
        std::vector<cv::Point3f> camPoints;
        std::vector<int8_t>      camCodes;       // same length as camPoints
        // Camera-frame floor points this keyframe observed WITHOUT a mask —
        // applyTo() erases any older tag these observations land on.
        std::vector<cv::Point3f> clearPoints;
        rtabmap::Transform       localTransform; // cam ↔ robot-base
        float                    zFloor = 0.0f;

        // Held only until labels arrive, then released to free memory.
        rtabmap::CameraModel     pendingCM;
        cv::Mat                  pendingDepth;   // CV_16UC1 or CV_32FC1; may be empty
        rtabmap::Transform       pendingPose;    // best pose at registerKeyframe() time
    };

    // Called from the SLAM thread when a new keyframe is created.
    // Snapshots all data needed to ray-cast labeled boxes later.
    void registerKeyframe(int nodeId,
                          const rtabmap::CameraModel & cm,
                          const cv::Mat & depth,
                          float zFloor,
                          const rtabmap::Transform & pose)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        Entry & e        = entries_[nodeId];
        e.localTransform = cm.localTransform();
        e.zFloor         = zFloor;
        e.pendingCM      = cm;
        e.pendingDepth   = depth.empty() ? cv::Mat() : depth.clone();
        e.pendingPose    = pose;
        e.camPoints.clear();
        e.camCodes.clear();
        e.clearPoints.clear();
    }

    // Called by SemanticWorker when /detect labels arrive for nodeId.
    //
    // If `mask` is non-empty (YOLO-seg returned a pixel union), the mask
    // defines the wall pixels directly — every non-zero pixel becomes code 1
    // (works because all current dataset classes resolve to wall; extend if
    // non-wall classes are ever added).
    // If `mask` is empty (mock mode, parse failure, no detections), falls
    // back to the legacy box raster path so the pipeline still produces
    // SOMETHING ray-castable.
    //
    // `detectorOk` distinguishes "YOLO ran and found nothing" (true — the
    // keyframe contributes clear votes) from "detector never ran: HTTP
    // failure, sidecar down" (false — no evidence either way; clear votes
    // here would erode true tags whenever the sidecar is offline).
    void setLabeledBoxes(int nodeId,
                         const std::vector<semantic::LabeledBox> & boxes,
                         const cv::Mat & mask = cv::Mat(),
                         bool detectorOk = true)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = entries_.find(nodeId);
        if (it == entries_.end())
        {
            UWARN("MaskStore[%d]: no entry — registerKeyframe missed?", nodeId);
            return;
        }
        Entry & e = it->second;

        if (!detectorOk)
        {
            e.pendingDepth = cv::Mat();
            e.pendingCM    = rtabmap::CameraModel();
            return;
        }

        if (e.pendingCM.fx() <= 0.0f)
        {
            UWARN("MaskStore[%d]: pendingCM not set — bailing", nodeId);
            return;
        }

        const int W = static_cast<int>(e.pendingCM.imageWidth());
        const int H = static_cast<int>(e.pendingCM.imageHeight());
        if (W <= 0 || H <= 0)
        {
            e.pendingDepth = cv::Mat();
            e.pendingCM    = rtabmap::CameraModel();
            return;
        }

        cv::Mat labeled = cv::Mat::zeros(H, W, CV_8UC1);
        int rasterised = 0;

        // Decide which detections have a wall-eligible label — used to gate
        // the mask path (no point rastering a mask when nothing in it is a
        // wall class).
        bool anyWall = false;
        for (const auto & lb : boxes)
        {
            if (resolveGridCode(lb) == 1) { anyWall = true; break; }
        }

        if (!mask.empty() && anyWall && mask.type() == CV_8UC1)
        {
            cv::Mat src;
            if (mask.cols == W && mask.rows == H)
                src = mask;
            else
                cv::resize(mask, src, cv::Size(W, H), 0, 0, cv::INTER_NEAREST);

            // No dilation here: it inflated every mask edge by a pixel and
            // amplified YOLO overshoot. Thin (1-2 px) lines survive coarse
            // sampling via the per-block scan in buildCamPoints instead.
            for (int v = 0; v < H; ++v)
            {
                const uchar * srow = src.ptr<uchar>(v);
                uchar * drow = labeled.ptr<uchar>(v);
                for (int u = 0; u < W; ++u)
                {
                    if (srow[u] > 0) drow[u] = 1;
                }
            }
            rasterised = 1;  // sentinel — "we have something to back-project"
        }
        else for (const auto & lb : boxes)
        {
            const int8_t code = resolveGridCode(lb);
            if (code == 0) continue;
            const int prio = priorityOf(code);

            const int x1 = std::max(0, std::min(W,     static_cast<int>(lb.x1)));
            const int y1 = std::max(0, std::min(H,     static_cast<int>(lb.y1)));
            const int x2 = std::max(0, std::min(W,     static_cast<int>(lb.x2)));
            const int y2 = std::max(0, std::min(H,     static_cast<int>(lb.y2)));
            if (x2 <= x1 || y2 <= y1) continue;

            for (int v = y1; v < y2; ++v)
            {
                uchar * row = labeled.ptr<uchar>(v);
                for (int u = x1; u < x2; ++u)
                {
                    if (priorityOf(static_cast<int8_t>(row[u])) < prio)
                        row[u] = static_cast<uchar>(code);
                }
            }
            ++rasterised;
        }

        const bool depthEmpty = e.pendingDepth.empty();
        const rtabmap::Transform T_cam2map = e.pendingPose * e.localTransform;

        // The zFloor snapshotted at registration (sensor grid viewpoint) is
        // typically 0, but the actual floor in MAP frame sits at
        // -cameraHeight (the Unity source has no translation in its local
        // transform) plus accumulated Z drift. With a wrong reference every
        // floor-proximity gate below rejects ALL samples (camPoints=0).
        // Estimate the floor from the depth image itself instead.
        float zEst = 0.0f;
        if (estimateFloorZ(e.pendingCM, e.pendingDepth, T_cam2map, zEst))
            e.zFloor = zEst;

        // Run even when nothing was rasterised (rasterised == 0): the clear
        // pass inside still emits negative-evidence samples for the floor
        // this keyframe observed mask-free.
        buildCamPoints(labeled, e.pendingCM, e.pendingDepth,
                       e.zFloor, T_cam2map,
                       e.camPoints, e.camCodes, e.clearPoints);

        UINFO("MaskStore[%d]: boxes=%zu anyWall=%d maskPath=%d rasterised=%d "
              "litPx=%d depthEmpty=%d camPoints=%zu clearPts=%zu zFloor=%.2f",
              nodeId, boxes.size(), (int)anyWall,
              (int)(!mask.empty() && anyWall && mask.type() == CV_8UC1),
              rasterised, cv::countNonZero(labeled), (int)depthEmpty,
              e.camPoints.size(), e.clearPoints.size(), e.zFloor);

        // Release bulky data — mask store no longer needs it for this entry.
        e.pendingDepth = cv::Mat();
        e.pendingCM    = rtabmap::CameraModel();
    }

    // Projects all stored camera-frame point clouds onto map8S using each
    // keyframe's CURRENT corrected pose. Only affine transforms — no
    // ray-casting — happen here.
    //
    // Latest observation wins (no voting): per cell, track the NEWEST
    // keyframe that saw it masked (wall) and the newest that observed it
    // mask-free (clear). If the clear observation is newer, the tag is
    // stale: it is not stamped and every wall sample on the cell is
    // permanently erased. Same-keyframe ties go to the mask, so a frame's
    // own clear lattice splat cannot kill the cells it just tagged.
    void applyTo(cv::Mat & map8S,
                 const std::map<int, rtabmap::Transform> & poses,
                 float xMin, float yMin, float cellSize)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (map8S.empty()) return;

        // Newest keyframe id observing each cell as wall / clear (-1 = never).
        // Keyframe ids increase monotonically, so id comparison == recency.
        cv::Mat lastWall  = cv::Mat(map8S.size(), CV_32SC1, cv::Scalar(-1));
        cv::Mat lastClear = cv::Mat(map8S.size(), CV_32SC1, cv::Scalar(-1));

        // Maps a camera-frame point to its grid cell with the same gating
        // everywhere. Returns false when the point does not land on a
        // valid floor cell.
        const auto cellOf = [&](const cv::Point3f & pCam,
                                const rtabmap::Transform & T_cam2map,
                                float zFloor, int & cx, int & cy) -> bool
        {
            const cv::Point3f Pmap = semantic::transformCamToMap(pCam, T_cam2map);
            // Reject if loop-closure correction moved the point off the floor.
            if (std::fabs(Pmap.z - zFloor) > 0.20f) return false;
            cx = static_cast<int>(std::floor((Pmap.x - xMin) / cellSize));
            cy = static_cast<int>(std::floor((Pmap.y - yMin) / cellSize));
            return cx >= 0 && cx < map8S.cols && cy >= 0 && cy < map8S.rows;
        };

        // Pass 1: record the newest wall / clear observation per cell.
        for (const auto & kv : entries_)
        {
            const Entry & e = kv.second;
            if (e.camPoints.empty() && e.clearPoints.empty()) continue;

            auto poseIt = poses.find(kv.first);
            if (poseIt == poses.end()) continue;

            const rtabmap::Transform T_cam2map =
                poseIt->second * e.localTransform;

            const size_t N = e.camPoints.size();
            for (size_t i = 0; i < N; ++i)
            {
                if (e.camCodes[i] == 0) continue;

                int cx, cy;
                if (!cellOf(e.camPoints[i], T_cam2map, e.zFloor, cx, cy)) continue;

                int32_t & lw = lastWall.at<int32_t>(cy, cx);
                if (kv.first > lw) lw = kv.first;
            }

            for (const cv::Point3f & cp : e.clearPoints)
            {
                int cx, cy;
                if (!cellOf(cp, T_cam2map, e.zFloor, cx, cy)) continue;

                // Clear samples sit on a coarse pixel lattice, so splat a
                // 3x3 cell neighborhood for contiguous coverage.
                for (int dy = -1; dy <= 1; ++dy)
                {
                    const int y = cy + dy;
                    if (y < 0 || y >= map8S.rows) continue;
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        const int x = cx + dx;
                        if (x < 0 || x >= map8S.cols) continue;
                        int32_t & lc = lastClear.at<int32_t>(y, x);
                        if (kv.first > lc) lc = kv.first;
                    }
                }
            }
        }

        // Pass 2: stale = the newest observation of a tagged cell is
        // mask-free (strictly newer: a tie means the same keyframe both
        // tagged the cell and grazed it with its clear lattice — the mask
        // wins).
        cv::Mat stale = cv::Mat::zeros(map8S.size(), CV_8UC1);
        bool anyStale = false;
        for (int y = 0; y < map8S.rows; ++y)
        {
            const int32_t * lwrow = lastWall.ptr<int32_t>(y);
            const int32_t * lcrow = lastClear.ptr<int32_t>(y);
            uchar * srow = stale.ptr<uchar>(y);
            for (int x = 0; x < map8S.cols; ++x)
            {
                if (lwrow[x] >= 0 && lcrow[x] > lwrow[x])
                {
                    srow[x] = 1;
                    anyStale = true;
                }
            }
        }

        // Pass 3: permanently erase wall samples on stale cells. Clear
        // samples stay — under latest-wins they cannot block a future
        // re-detection (a newer wall observation always outranks them).
        if (anyStale)
        {
            size_t prunedWall = 0;
            for (auto & kv : entries_)
            {
                Entry & e = kv.second;
                if (e.camPoints.empty()) continue;

                auto poseIt = poses.find(kv.first);
                if (poseIt == poses.end()) continue;

                const rtabmap::Transform T_cam2map =
                    poseIt->second * e.localTransform;

                size_t w = 0;
                for (size_t i = 0; i < e.camPoints.size(); ++i)
                {
                    int cx, cy;
                    const bool drop =
                        e.camCodes[i] != 0 &&
                        cellOf(e.camPoints[i], T_cam2map, e.zFloor, cx, cy) &&
                        stale.at<uchar>(cy, cx) != 0;
                    if (drop) { ++prunedWall; continue; }
                    e.camPoints[w] = e.camPoints[i];
                    e.camCodes[w]  = e.camCodes[i];
                    ++w;
                }
                e.camPoints.resize(w);
                e.camCodes.resize(w);
            }
            UINFO("MaskStore: erased %zu wall samples on cells observed mask-free",
                  prunedWall);
        }

        // Final stamp. Only code 1 (wall) is in use.
        for (int y = 0; y < map8S.rows; ++y)
        {
            const int32_t * lwrow = lastWall.ptr<int32_t>(y);
            const uchar * srow = stale.ptr<uchar>(y);
            int8_t * mrow = map8S.ptr<int8_t>(y);
            for (int x = 0; x < map8S.cols; ++x)
            {
                if (lwrow[x] < 0 || srow[x] != 0) continue;
                if (priorityOf(mrow[x]) < priorityOf(1))
                    mrow[x] = 1;
            }
        }
    }

    size_t entryCount() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return entries_.size();
    }

    size_t maskedCount() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        size_t n = 0;
        for (const auto & kv : entries_)
            if (!kv.second.camPoints.empty()) ++n;
        return n;
    }

private:
    // YOLO label -> int8 grid code. Returning 0 means "do not write".
    static int8_t resolveGridCode(const semantic::LabeledBox & lb)
    {
        const std::string & l = lb.label;
        if (l == "centerLine" || l == "parkingLine") return 1;
        return 0;
    }

    // Priority for rasterisation overlap: higher overrides lower. Only the
    // wall code is in use; RTABMap's -1 unknown / 0 free / 100 occupied
    // read as priority 0 so wall still wins.
    static int priorityOf(int8_t code)
    {
        if (code == 1) return 100;  // wall-like
        return 0;
    }

    static float readDepthMeters(const cv::Mat & depth, int u, int v)
    {
        if (depth.type() == CV_16UC1) return depth.at<uint16_t>(v, u) * 0.001f;
        if (depth.type() == CV_32FC1) return depth.at<float>(v, u);
        return 0.0f;
    }

    // Robust floor-height estimate in MAP frame: project a coarse lattice
    // of the lower image half and take the 20th percentile of z — low
    // enough to sit on the floor, robust against obstacles standing on it.
    // Returns false (caller keeps the registered zFloor) when depth is
    // missing or too few samples are valid.
    static bool estimateFloorZ(
        const rtabmap::CameraModel & cm,
        const cv::Mat & depth,
        const rtabmap::Transform & T_cam2map,
        float & zOut)
    {
        if (depth.empty()) return false;

        const float fx = static_cast<float>(cm.fx());
        const float fy = static_cast<float>(cm.fy());
        const float cx = static_cast<float>(cm.cx());
        const float cy = static_cast<float>(cm.cy());

        std::vector<float> zs;
        zs.reserve(512);
        const int step = std::max(8, depth.cols / 80);
        for (int v = depth.rows / 2; v < depth.rows; v += step)
        {
            for (int u = 0; u < depth.cols; u += step)
            {
                const float d = readDepthMeters(depth, u, v);
                if (d < 0.3f || d > kMaxSampleRange) continue;
                const cv::Point3f Pcam = semantic::backprojectDepth(
                    static_cast<float>(u), static_cast<float>(v),
                    d, fx, fy, cx, cy);
                zs.push_back(semantic::transformCamToMap(Pcam, T_cam2map).z);
            }
        }
        if (zs.size() < 50) return false;

        const size_t k = zs.size() / 5;
        std::nth_element(zs.begin(), zs.begin() + k, zs.end());
        zOut = zs[k];
        return true;
    }

    // Ray-cast: walks the labeled mask, projects every non-zero pixel onto
    // the floor plane, and emits parallel camera-frame point + code arrays.
    // Also emits sparse clear samples (floor observed with valid depth, no
    // mask) used as negative evidence by applyTo().
    static void buildCamPoints(
        const cv::Mat & labeledMask,
        const rtabmap::CameraModel & cm,
        const cv::Mat & depth,
        float zFloor,
        const rtabmap::Transform & T_cam2map,
        std::vector<cv::Point3f> & outPts,
        std::vector<int8_t>      & outCodes,
        std::vector<cv::Point3f> & outClearPts)
    {
        const float fx = static_cast<float>(cm.fx());
        const float fy = static_cast<float>(cm.fy());
        const float cx = static_cast<float>(cm.cx());
        const float cy = static_cast<float>(cm.cy());

        outPts.clear();
        outCodes.clear();
        outClearPts.clear();
        outPts.reserve(512);
        outCodes.reserve(512);

        // entries_ accumulates one Entry per keyframe forever (camera-frame
        // points are kept so loop-closure can move them via applyTo). To
        // bound RAM over long sessions, cap raster sample count per entry —
        // base step=2 for normal keyframes, escalate proportionally if the
        // raster covers more than kMaxPointsPerEntry pixels.
        int step = 2;
        const int totalLit = cv::countNonZero(labeledMask);
        if (totalLit > kMaxPointsPerEntry)
        {
            const float ratio = std::sqrt(
                static_cast<float>(totalLit) /
                static_cast<float>(kMaxPointsPerEntry));
            step = std::max(2, static_cast<int>(std::ceil(ratio)));
        }
        if (totalLit > 0)
        for (int v = 0; v < labeledMask.rows; v += step)
        {
            const int vEnd = std::min(labeledMask.rows, v + step);
            for (int u = 0; u < labeledMask.cols; u += step)
            {
                // Scan the whole step x step block for a lit pixel so thin
                // (1-2 px) lines survive coarse sampling. This replaces the
                // old mask dilation, which inflated every edge and amplified
                // YOLO overshoot — here the point is emitted at the lit
                // pixel's true position instead.
                const int uEnd = std::min(labeledMask.cols, u + step);
                int lu = -1, lv = -1;
                int8_t code = 0;
                for (int bv = v; bv < vEnd && code == 0; ++bv)
                {
                    const uchar * row = labeledMask.ptr<uchar>(bv);
                    for (int bu = u; bu < uEnd; ++bu)
                    {
                        if (row[bu] != 0)
                        {
                            code = static_cast<int8_t>(row[bu]);
                            lu = bu;
                            lv = bv;
                            break;
                        }
                    }
                }
                if (code == 0) continue;

                cv::Point3f Pcam;
                bool ok = false;

                // Path 1: depth-based (pose-independent)
                if (!depth.empty() &&
                    lv < depth.rows && lu < depth.cols)
                {
                    const float d = readDepthMeters(depth, lu, lv);
                    if (d >= 0.3f && d <= kMaxSampleRange)
                    {
                        Pcam = semantic::backprojectDepth(
                            static_cast<float>(lu), static_cast<float>(lv),
                            d, fx, fy, cx, cy);
                        ok = true;
                    }
                }

                // Path 2: ray-plane fallback using the pose at registration time.
                if (!ok)
                {
                    cv::Point3f Pmap;
                    ok = semantic::rayPlaneIntersect(
                        static_cast<float>(lu), static_cast<float>(lv),
                        fx, fy, cx, cy, T_cam2map, zFloor, Pmap);
                    if (ok)
                    {
                        const rtabmap::Transform T_map2cam = T_cam2map.inverse();
                        Pcam = semantic::transformCamToMap(Pmap, T_map2cam);
                        // Near-horizon pixels intersect the floor far away,
                        // so a mask spilling a few pixels upward would tag
                        // cells meters from the sensor. Apply the same range
                        // cap as the depth path.
                        const float range = std::sqrt(
                            Pcam.x*Pcam.x + Pcam.y*Pcam.y + Pcam.z*Pcam.z);
                        if (range > kMaxSampleRange) ok = false;
                    }
                }

                if (!ok) continue;

                // Validate: project with current pose, check floor proximity.
                const cv::Point3f Pmap_check =
                    semantic::transformCamToMap(Pcam, T_cam2map);
                if (std::fabs(Pmap_check.z - zFloor) > 0.15f) continue;

                outPts.push_back(Pcam);
                outCodes.push_back(code);
            }
        }

        // Clear samples: coarse lattice over pixels that observed the floor
        // (valid depth landing near zFloor) without any mask. Depth-only on
        // purpose — without a depth reading we cannot know the ray actually
        // reached the floor (it may hit an obstacle first), and a wrong
        // clear vote erases true tags.
        if (!depth.empty())
        {
            const int W = labeledMask.cols;
            const int H = labeledMask.rows;
            const int clearStep = std::max(4, static_cast<int>(std::ceil(
                std::sqrt(static_cast<float>(W * H) /
                          static_cast<float>(kMaxClearPerEntry)))));
            outClearPts.reserve(kMaxClearPerEntry);
            for (int v = 0; v < H && v < depth.rows; v += clearStep)
            {
                const uchar * row = labeledMask.ptr<uchar>(v);
                for (int u = 0; u < W && u < depth.cols; u += clearStep)
                {
                    if (row[u] != 0) continue;
                    const float d = readDepthMeters(depth, u, v);
                    if (d < 0.3f || d > kMaxSampleRange) continue;
                    const cv::Point3f Pcam = semantic::backprojectDepth(
                        static_cast<float>(u), static_cast<float>(v),
                        d, fx, fy, cx, cy);
                    const cv::Point3f Pmap =
                        semantic::transformCamToMap(Pcam, T_cam2map);
                    if (std::fabs(Pmap.z - zFloor) > 0.15f) continue;
                    outClearPts.push_back(Pcam);
                }
            }
        }
    }

    // Per-entry raster sample budget. Each kept pixel becomes one cv::Point3f
    // (12B) + int8_t code (1B) = 13B, plus vector overhead. At 4096 →
    // ~53 KB/entry → ~270 MB for 5k keyframes (~1 hr SLAM at 1.4 Hz keyframe
    // rate). Bumping this trades RAM for finer semantic detail.
    static constexpr int kMaxPointsPerEntry = 4096;

    // Clear-sample budget per entry (12 B each → ~12 KB/entry, paid for
    // EVERY keyframe, masked or not — ~60 MB for 5k keyframes). Drives the
    // clear lattice spacing in buildCamPoints.
    static constexpr int kMaxClearPerEntry = 1024;

    // Shared range cap for depth reads and the ray-plane fallback. Beyond
    // this D455 depth is mostly noise and floor intersections are the worst
    // overshoot offenders.
    static constexpr float kMaxSampleRange = 6.0f;

    mutable std::mutex                    mtx_;
    std::unordered_map<int, Entry>        entries_;
};

#endif /* SEMANTICMASKSTORE_H_ */
