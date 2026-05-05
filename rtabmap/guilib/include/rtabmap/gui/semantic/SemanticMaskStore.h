/*
 * SemanticMaskStore.h
 *
 * Per-keyframe semantic point cache, anchored by RTAB-Map nodeId.
 *
 * Key design principle: ray-cast (depth read / ray-plane intersection) is
 * pose-independent — it operates in camera space and depends only on the
 * depth image and intrinsics.  It is therefore done ONCE when the labeled
 * boxes arrive (setLabeledBoxes) and the resulting camera-frame 3-D points
 * are stored, each tagged with the grid code derived from its Gemini label.
 *
 * applyTo() then only needs to apply the current corrected pose transform
 * (a cheap affine multiply) to project those points onto the map grid.
 * Loop-closure pose corrections flow through automatically because poses
 * are looked up fresh on every applyTo() call.
 *
 * Grid codes (semantic overlay on int8 occupancy grid; matches prompt.md):
 *   1      wall-like        pillar, traffic_cone, parking_line,
 *                           no_entry_sign, construction_sign
 *   10     destination      exit_area
 *   11..18 dest direction   exit_sign, floor_arrow (10 + worldDirBin)
 *   21..28 one-way zone     one_way_marker (20 + worldDirBin)
 *   0      skip              lane_divider, intersection, anything else
 *
 * Direction bins (from semantic::imageAngleToWorldDirBin):
 *   bin 1 = +Y world (north),  bin 3 = +X (east),
 *   bin 5 = -Y (south),        bin 7 = -X (west); CW from north.
 *
 * Direction codes are baked at setLabeledBoxes() time using the keyframe's
 * registration pose. Subsequent loop-closure rotations do NOT re-quantize
 * already-stored bins — acceptable while quantization is 45° coarse.
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

    // Called by SemanticWorker when /classify_batch labels arrive for nodeId.
    // Rasterises the boxes into a label map, performs the ray-cast, and stores
    // (camera-frame point, grid code) pairs. Releases pending depth + camera
    // model. Boxes whose label maps to grid code 0 are silently skipped.
    void setLabeledBoxes(int nodeId,
                         const std::vector<semantic::LabeledBox> & boxes)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = entries_.find(nodeId);
        if (it == entries_.end()) return;
        Entry & e = it->second;

        if (e.pendingCM.fx() <= 0.0f)
        {
            // Camera model never registered (or already consumed) — bail.
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

        // World-frame yaw at registration; used to quantize directional
        // labels' image-plane visualAngle into world-frame direction bins.
        const float robotYaw = e.pendingPose.theta();

        cv::Mat labeled = cv::Mat::zeros(H, W, CV_8UC1);
        int rasterised = 0;
        for (const auto & lb : boxes)
        {
            const int8_t code = resolveGridCode(lb, robotYaw);
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

        if (rasterised > 0)
        {
            const rtabmap::Transform T_cam2map = e.pendingPose * e.localTransform;
            buildCamPoints(labeled, e.pendingCM, e.pendingDepth,
                           e.zFloor, T_cam2map,
                           e.camPoints, e.camCodes);
        }

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
    // LLM label + robot yaw -> int8 grid code (prompt.md catalog).
    // Returning 0 means "do not write". Directional labels require a
    // valid lb.visualAngle; otherwise they are skipped.
    static int8_t resolveGridCode(const semantic::LabeledBox & lb,
                                  float robotYawRad)
    {
        const std::string & l = lb.label;
        if (l == "pillar"            ||
            l == "traffic_cone"      ||
            l == "parking_line"      ||
            l == "no_entry_sign"     ||
            l == "construction_sign")  return 1;
        if (l == "exit_area")          return 10;

        if (lb.visualAngle < 0.0f) return 0;  // direction labels need angle

        if (l == "exit_sign" || l == "floor_arrow")
        {
            const int bin = semantic::imageAngleToWorldDirBin(
                lb.visualAngle, robotYawRad);
            return static_cast<int8_t>(10 + bin);   // 11..18
        }
        if (l == "one_way_marker")
        {
            const int bin = semantic::imageAngleToWorldDirBin(
                lb.visualAngle, robotYawRad);
            return static_cast<int8_t>(20 + bin);   // 21..28
        }
        // lane_divider, intersection, unknown -> skip.
        return 0;
    }

    // Priority for rasterisation overlap: higher overrides lower. Values
    // outside the semantic set (e.g. RTABMap's -1 unknown, 0 free, 100
    // occupied) read as priority 0 so wall codes still win over them.
    static int priorityOf(int8_t code)
    {
        if (code == 1)                         return 100;  // wall-like
        if (code >= 21 && code <= 28)          return  60;  // one-way rule
        if (code >= 11 && code <= 18)          return  55;  // dest direction
        if (code == 10)                        return  50;  // exit area
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

        const int step = 2;

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

    mutable std::mutex                    mtx_;
    std::unordered_map<int, Entry>        entries_;
};

#endif /* SEMANTICMASKSTORE_H_ */
