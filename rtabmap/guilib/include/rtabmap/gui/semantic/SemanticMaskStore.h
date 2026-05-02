/*
 * SemanticMaskStore.h
 *
 * Per-keyframe semantic point cache, anchored by RTAB-Map nodeId.
 *
 * Key design principle: ray-cast (depth read / ray-plane intersection) is
 * pose-independent — it operates in camera space and depends only on the
 * depth image and intrinsics.  It is therefore done ONCE when the mask
 * arrives (setMask) and the resulting camera-frame 3-D points are stored.
 *
 * applyTo() then only needs to apply the current corrected pose transform
 * (a cheap affine multiply) to project those points onto the map grid.
 * Loop-closure pose corrections flow through automatically because poses
 * are looked up fresh on every applyTo() call.
 *
 * Complexity:
 *   Old: applyTo = O(N * W*H/step^2) with full ray-cast per pixel per call
 *   New: setMask = O(W*H/step^2)  ray-cast once
 *        applyTo = O(N * M)  affine transform only  (M = nonzero pixel count)
 */

#ifndef SEMANTICMASKSTORE_H_
#define SEMANTICMASKSTORE_H_

#include "SemanticBackproject.h"

#include "rtabmap/core/Transform.h"
#include "rtabmap/core/CameraModel.h"

#include <opencv2/core.hpp>
#include <unordered_map>
#include <map>
#include <mutex>
#include <cmath>
#include <vector>

class SemanticMaskStore
{
public:
    struct Entry {
        // Camera-frame 3-D points computed once at setMask() time.
        // These are pose-independent; only the transform in applyTo() varies.
        std::vector<cv::Point3f> camPoints;
        rtabmap::Transform       localTransform; // cam ↔ robot-base (from CameraModel)
        float                    zFloor = 0.0f;

        // Held only until the mask arrives, then released to free memory.
        rtabmap::CameraModel     pendingCM;
        cv::Mat                  pendingDepth;   // CV_16UC1 or CV_32FC1; may be empty
        rtabmap::Transform       pendingPose;    // best pose at registerKeyframe() time
    };

    // Called from the SLAM thread when a new keyframe is created.
    // Snapshots all data needed to ray-cast the mask later.
    void registerKeyframe(int nodeId,
                          const rtabmap::CameraModel & cm,
                          const cv::Mat & depth,
                          float zFloor,
                          const rtabmap::Transform & pose)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        Entry & e       = entries_[nodeId];
        e.localTransform = cm.localTransform();
        e.zFloor         = zFloor;
        e.pendingCM      = cm;
        e.pendingDepth   = depth.empty() ? cv::Mat() : depth.clone();
        e.pendingPose    = pose;
        e.camPoints.clear(); // reset if re-registered
    }

    // Called by SemanticWorker when a SAM mask arrives for nodeId.
    // Performs the ray-cast immediately and stores camera-frame points.
    // Releases depth image and camera model to free memory.
    void setMask(int nodeId, const cv::Mat & mask)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = entries_.find(nodeId);
        if (it == entries_.end()) return;
        Entry & e = it->second;

        if (e.pendingCM.fx() <= 0.0f || mask.empty()) return;

        const rtabmap::Transform T_cam2map =
            e.pendingPose * e.localTransform;

        e.camPoints = buildCamPoints(mask, e.pendingCM, e.pendingDepth,
                                     e.zFloor, T_cam2map);

        // Release bulky data — no longer needed.
        e.pendingDepth = cv::Mat();
        e.pendingCM    = rtabmap::CameraModel();
    }

    // Projects all stored camera-frame point clouds onto map8S using each
    // keyframe's CURRENT corrected pose.  Only affine transforms — no
    // ray-casting — happen here.
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

            for (const cv::Point3f & Pcam : e.camPoints)
            {
                const cv::Point3f Pmap =
                    semantic::transformCamToMap(Pcam, T_cam2map);

                // Reject if loop-closure correction moved point off the floor.
                if (std::fabs(Pmap.z - e.zFloor) > 0.20f) continue;

                const int cx = static_cast<int>(
                    std::floor((Pmap.x - xMin) / cellSize));
                const int cy = static_cast<int>(
                    std::floor((Pmap.y - yMin) / cellSize));
                if (cx < 0 || cx >= map8S.cols) continue;
                if (cy < 0 || cy >= map8S.rows) continue;

                map8S.at<int8_t>(cy, cx) = 80;
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
    static float readDepthMeters(const cv::Mat & depth, int u, int v)
    {
        if (depth.type() == CV_16UC1) return depth.at<uint16_t>(v, u) * 0.001f;
        if (depth.type() == CV_32FC1) return depth.at<float>(v, u);
        return 0.0f;
    }

    // Ray-cast: called once per keyframe when the mask arrives.
    // Returns camera-frame 3-D points for every nonzero mask pixel that
    // successfully projects onto the floor plane.
    static std::vector<cv::Point3f> buildCamPoints(
        const cv::Mat & mask,
        const rtabmap::CameraModel & cm,
        const cv::Mat & depth,
        float zFloor,
        const rtabmap::Transform & T_cam2map)
    {
        const float fx = static_cast<float>(cm.fx());
        const float fy = static_cast<float>(cm.fy());
        const float cx = static_cast<float>(cm.cx());
        const float cy = static_cast<float>(cm.cy());

        std::vector<cv::Point3f> pts;
        pts.reserve(512);

        const int step = 2;

        for (int v = 0; v < mask.rows; v += step)
        {
            const uchar * row = mask.ptr<uchar>(v);
            for (int u = 0; u < mask.cols; u += step)
            {
                if (row[u] == 0) continue;

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

                // Path 2: ray-plane fallback using the pose at registration
                // time.  Slight error if loop closure later corrects the pose,
                // but floor points are robust to small drift.
                if (!ok)
                {
                    cv::Point3f Pmap;
                    ok = semantic::rayPlaneIntersect(
                        static_cast<float>(u), static_cast<float>(v),
                        fx, fy, cx, cy, T_cam2map, zFloor, Pmap);
                    if (ok)
                    {
                        // Convert back to camera frame so applyTo only needs
                        // a forward transform.
                        const rtabmap::Transform T_map2cam = T_cam2map.inverse();
                        Pcam = semantic::transformCamToMap(Pmap, T_map2cam);
                    }
                }

                if (!ok) continue;

                // Validate: project with current pose and check floor proximity.
                const cv::Point3f Pmap_check =
                    semantic::transformCamToMap(Pcam, T_cam2map);
                if (std::fabs(Pmap_check.z - zFloor) > 0.15f) continue;

                pts.push_back(Pcam);
            }
        }

        return pts;
    }

    mutable std::mutex                    mtx_;
    std::unordered_map<int, Entry>        entries_;
};

#endif /* SEMANTICMASKSTORE_H_ */
