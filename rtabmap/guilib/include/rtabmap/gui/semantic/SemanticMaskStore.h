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
    void setLabeledBoxes(int nodeId,
                         const std::vector<semantic::LabeledBox> & boxes,
                         const cv::Mat & mask = cv::Mat())
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = entries_.find(nodeId);
        if (it == entries_.end())
        {
            UWARN("MaskStore[%d]: no entry — registerKeyframe missed?", nodeId);
            return;
        }
        Entry & e = it->second;

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

            // Thicken thin lines (parkingLine masks are often 1-2 px wide)
            // so the step=2 back-projection in buildCamPoints catches them.
            // Default 3x3 kernel, 1 iteration → 1 px line becomes ~3 px.
            cv::dilate(src, src, cv::Mat());

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
        if (rasterised > 0)
        {
            const rtabmap::Transform T_cam2map = e.pendingPose * e.localTransform;
            buildCamPoints(labeled, e.pendingCM, e.pendingDepth,
                           e.zFloor, T_cam2map,
                           e.camPoints, e.camCodes);
        }

        UINFO("MaskStore[%d]: boxes=%zu anyWall=%d maskPath=%d "
              "litPx=%d depthEmpty=%d camPoints=%zu zFloor=%.2f",
              nodeId, boxes.size(), (int)anyWall,
              (int)(!mask.empty() && anyWall && mask.type() == CV_8UC1),
              cv::countNonZero(labeled), (int)depthEmpty,
              e.camPoints.size(), e.zFloor);

        // Release bulky data — mask store no longer needs it for this entry.
        e.pendingDepth = cv::Mat();
        e.pendingCM    = rtabmap::CameraModel();
    }

    // Projects all stored camera-frame point clouds onto map8S using each
    // keyframe's CURRENT corrected pose. Only affine transforms — no
    // ray-casting — happen here. Higher-priority codes (wall) cannot be
    // overwritten by lower-priority codes (destination) within a single
    // applyTo() call.
    void applyTo(cv::Mat & map8S,
                 const std::map<int, rtabmap::Transform> & poses,
                 float xMin, float yMin, float cellSize) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        for (const auto & kv : entries_)
        {
            const Entry & e = kv.second;
            if (e.camPoints.empty()) continue;

            auto poseIt = poses.find(kv.first);
            if (poseIt == poses.end()) continue;

            const rtabmap::Transform T_cam2map =
                poseIt->second * e.localTransform;

            const size_t N = e.camPoints.size();
            for (size_t i = 0; i < N; ++i)
            {
                const int8_t code = e.camCodes[i];
                if (code == 0) continue;

                const cv::Point3f Pmap =
                    semantic::transformCamToMap(e.camPoints[i], T_cam2map);

                // Reject if loop-closure correction moved the point off the floor.
                if (std::fabs(Pmap.z - e.zFloor) > 0.20f) continue;

                const int cx = static_cast<int>(
                    std::floor((Pmap.x - xMin) / cellSize));
                const int cy = static_cast<int>(
                    std::floor((Pmap.y - yMin) / cellSize));
                if (cx < 0 || cx >= map8S.cols) continue;
                if (cy < 0 || cy >= map8S.rows) continue;

                int8_t & cell = map8S.at<int8_t>(cy, cx);
                if (priorityOf(cell) < priorityOf(code))
                    cell = code;
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

    // Ray-cast: walks the labeled mask, projects every non-zero pixel onto
    // the floor plane, and emits parallel camera-frame point + code arrays.
    static void buildCamPoints(
        const cv::Mat & labeledMask,
        const rtabmap::CameraModel & cm,
        const cv::Mat & depth,
        float zFloor,
        const rtabmap::Transform & T_cam2map,
        std::vector<cv::Point3f> & outPts,
        std::vector<int8_t>      & outCodes)
    {
        const float fx = static_cast<float>(cm.fx());
        const float fy = static_cast<float>(cm.fy());
        const float cx = static_cast<float>(cm.cx());
        const float cy = static_cast<float>(cm.cy());

        outPts.clear();
        outCodes.clear();
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
        for (int v = 0; v < labeledMask.rows; v += step)
        {
            const uchar * row = labeledMask.ptr<uchar>(v);
            for (int u = 0; u < labeledMask.cols; u += step)
            {
                const int8_t code = static_cast<int8_t>(row[u]);
                if (code == 0) continue;

                cv::Point3f Pcam;
                bool ok = false;

                // Path 1: depth-based (pose-independent)
                if (!depth.empty() &&
                    v < depth.rows && u < depth.cols)
                {
                    const float d = readDepthMeters(depth, u, v);
                    if (d >= 0.3f && d <= 6.0f)
                    {
                        Pcam = semantic::backprojectDepth(
                            static_cast<float>(u), static_cast<float>(v),
                            d, fx, fy, cx, cy);
                        ok = true;
                    }
                }

                // Path 2: ray-plane fallback using the pose at registration time.
                if (!ok)
                {
                    cv::Point3f Pmap;
                    ok = semantic::rayPlaneIntersect(
                        static_cast<float>(u), static_cast<float>(v),
                        fx, fy, cx, cy, T_cam2map, zFloor, Pmap);
                    if (ok)
                    {
                        const rtabmap::Transform T_map2cam = T_cam2map.inverse();
                        Pcam = semantic::transformCamToMap(Pmap, T_map2cam);
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
    }

    // Per-entry raster sample budget. Each kept pixel becomes one cv::Point3f
    // (12B) + int8_t code (1B) = 13B, plus vector overhead. At 4096 →
    // ~53 KB/entry → ~270 MB for 5k keyframes (~1 hr SLAM at 1.4 Hz keyframe
    // rate). Bumping this trades RAM for finer semantic detail.
    static constexpr int kMaxPointsPerEntry = 4096;

    mutable std::mutex                    mtx_;
    std::unordered_map<int, Entry>        entries_;
};

#endif /* SEMANTICMASKSTORE_H_ */
