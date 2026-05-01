/*
 * SemanticBackproject.h
 *
 * Pure header functions for projecting image-space pixels onto the world map.
 * No state, no allocations beyond the cv::Point3f return.
 */

#ifndef SEMANTICBACKPROJECT_H_
#define SEMANTICBACKPROJECT_H_

#include "rtabmap/core/Transform.h"
#include <opencv2/core.hpp>
#include <cmath>

namespace semantic {

inline cv::Point3f backprojectDepth(
    float u, float v, float d,
    float fx, float fy, float cx, float cy)
{
    return cv::Point3f((u - cx) * d / fx,
                       (v - cy) * d / fy,
                       d);
}

inline cv::Point3f transformCamToMap(
    const cv::Point3f & p,
    const rtabmap::Transform & T_cam2map)
{
    const float x = T_cam2map.r11()*p.x + T_cam2map.r12()*p.y + T_cam2map.r13()*p.z + T_cam2map.x();
    const float y = T_cam2map.r21()*p.x + T_cam2map.r22()*p.y + T_cam2map.r23()*p.z + T_cam2map.y();
    const float z = T_cam2map.r31()*p.x + T_cam2map.r32()*p.y + T_cam2map.r33()*p.z + T_cam2map.z();
    return cv::Point3f(x, y, z);
}

// Ray-plane intersection in MAP frame.
// Returns false if ray is parallel to floor or intersects behind the camera.
inline bool rayPlaneIntersect(
    float u, float v,
    float fx, float fy, float cx, float cy,
    const rtabmap::Transform & T_cam2map,
    float zFloor,
    cv::Point3f & outMap)
{
    // Ray in camera frame
    const float rx = (u - cx) / fx;
    const float ry = (v - cy) / fy;
    const float rz = 1.0f;

    // Rotate ray to map frame (translation does not affect direction)
    const float rmx = T_cam2map.r11()*rx + T_cam2map.r12()*ry + T_cam2map.r13()*rz;
    const float rmy = T_cam2map.r21()*rx + T_cam2map.r22()*ry + T_cam2map.r23()*rz;
    const float rmz = T_cam2map.r31()*rx + T_cam2map.r32()*ry + T_cam2map.r33()*rz;

    if (std::fabs(rmz) < 1e-6f) return false;   // ray ∥ floor

    const float ox = T_cam2map.x();
    const float oy = T_cam2map.y();
    const float oz = T_cam2map.z();

    const float t = (zFloor - oz) / rmz;
    if (t <= 0.0f) return false;                // intersection behind camera

    outMap.x = ox + t * rmx;
    outMap.y = oy + t * rmy;
    outMap.z = zFloor;
    return true;
}

}  // namespace semantic

#endif /* SEMANTICBACKPROJECT_H_ */
