/*
 * SemanticMaskStore.h
 *
 * Per-keyframe semantic mask cache, anchored by RTAB-Map nodeId.
 * On each publish, applyTo() rasters all stored masks onto the global map
 * using the keyframe's CURRENT corrected pose from stats.poses().
 * Loop closure → keyframe poses jump → next applyTo reflects the jump
 * automatically (no re-anchoring needed).
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

class SemanticMaskStore
{
public:
    struct Entry {
        rtabmap::CameraModel cameraModel;   // fx,fy,cx,cy + localTransform (cam ↔ base)
        cv::Mat              depth;          // CV_16UC1 (mm) or CV_32FC1 (m); may be empty
        float                zFloor = 0.0f;  // gridViewPoint().z of this keyframe
        cv::Mat              mask;           // CV_8UC1 (0/255); empty until worker fills it
    };

    // Called on new-keyframe detection from the SLAM thread.
    // Snapshots the camera model + depth + floor height so we can back-project later
    // even if the SensorData has been evicted from working memory.
    void registerKeyframe(int nodeId,
                          const rtabmap::CameraModel & cm,
                          const cv::Mat & depth,
                          float zFloor)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        Entry & e = entries_[nodeId];
        e.cameraModel = cm;
        e.depth       = depth.empty() ? cv::Mat() : depth.clone();
        e.zFloor      = zFloor;
    }

    // Called by SemanticWorker thread when a detection result arrives.
    // Silently dropped if the keyframe was never registered (race / forgotten).
    void setMask(int nodeId, const cv::Mat & mask)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = entries_.find(nodeId);
        if (it == entries_.end()) return;
        it->second.mask = mask.clone();
    }

    // Called from the publish path, AFTER grid_.getMap(...).
    // Mutates map8S in place: cells covered by any mask are forced to 100.
    void applyTo(cv::Mat & map8S,
                 const std::map<int, rtabmap::Transform> & poses,
                 float xMin, float yMin, float cellSize) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        for (const auto & kv : entries_)
        {
            const int nodeId = kv.first;
            const Entry & e  = kv.second;
            if (e.mask.empty()) continue;

            auto poseIt = poses.find(nodeId);
            if (poseIt == poses.end()) continue;   // keyframe not in current pose graph

            const rtabmap::Transform T_cam2map =
                poseIt->second * e.cameraModel.localTransform();

            rasterMask(e, T_cam2map, map8S, xMin, yMin, cellSize);
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
            if (!kv.second.mask.empty()) ++n;
        return n;
    }

private:
    static float readDepthMeters(const cv::Mat & depth, int u, int v)
    {
        if (depth.type() == CV_16UC1) return depth.at<uint16_t>(v, u) * 0.001f;
        if (depth.type() == CV_32FC1) return depth.at<float>(v, u);
        return 0.0f;
    }

    static void rasterMask(const Entry & e,
                           const rtabmap::Transform & T_cam2map,
                           cv::Mat & map8S,
                           float xMin, float yMin, float cellSize)
    {
        const cv::Mat & mask = e.mask;
        const float fx = static_cast<float>(e.cameraModel.fx());
        const float fy = static_cast<float>(e.cameraModel.fy());
        const float cx = static_cast<float>(e.cameraModel.cx());
        const float cy = static_cast<float>(e.cameraModel.cy());
        const float zFloor = e.zFloor;

        if (fx <= 0.0f || fy <= 0.0f) return;

        // Subsample to keep raster cost bounded; we only need cell-level resolution.
        const int step = 2;

        for (int v = 0; v < mask.rows; v += step)
        {
            const uchar * row = mask.ptr<uchar>(v);
            for (int u = 0; u < mask.cols; u += step)
            {
                if (row[u] == 0) continue;

                cv::Point3f Pmap;
                bool ok = false;

                // Path 1: depth-based back-projection
                if (!e.depth.empty() &&
                    v < e.depth.rows && u < e.depth.cols)
                {
                    const float d = readDepthMeters(e.depth, u, v);
                    if (d >= 0.3f && d <= 6.0f)
                    {
                        const cv::Point3f Pcam =
                            semantic::backprojectDepth(static_cast<float>(u),
                                                       static_cast<float>(v),
                                                       d, fx, fy, cx, cy);
                        Pmap = semantic::transformCamToMap(Pcam, T_cam2map);
                        ok = true;
                    }
                }

                // Path 2: ray-plane fallback (depth hole on painted floor)
                if (!ok)
                {
                    ok = semantic::rayPlaneIntersect(static_cast<float>(u),
                                                     static_cast<float>(v),
                                                     fx, fy, cx, cy,
                                                     T_cam2map, zFloor, Pmap);
                }
                if (!ok) continue;

                // Sanity: reject anything significantly off the floor plane
                if (std::fabs(Pmap.z - zFloor) > 0.15f) continue;

                // World → cell index
                const int cxIdx = static_cast<int>(std::floor((Pmap.x - xMin) / cellSize));
                const int cyIdx = static_cast<int>(std::floor((Pmap.y - yMin) / cellSize));
                if (cxIdx < 0 || cxIdx >= map8S.cols) continue;
                if (cyIdx < 0 || cyIdx >= map8S.rows) continue;

                // Force obstacle, even if RTAB-Map marked the cell as free.
                map8S.at<int8_t>(cyIdx, cxIdx) = 100;
            }
        }
    }

    mutable std::mutex                       mtx_;
    std::unordered_map<int, Entry>           entries_;
};

#endif /* SEMANTICMASKSTORE_H_ */
