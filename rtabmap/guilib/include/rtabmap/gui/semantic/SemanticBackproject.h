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

// Convert an image-plane visual angle (LLM output) into a world-frame
// direction bin in {1..8}, following the prompt.md convention:
//   bin 1 = +Y_world (north), bin 3 = +X_world (east),
//   bin 5 = -Y_world (south), bin 7 = -X_world (west); CW from north.
//
// Inputs:
//   visualAngleDeg : [0, 360), CW from image-up. 0 = arrow points to image
//                    top, 90 = points right, 180 = points down, 270 = left.
//   robotYawRad    : robot heading, CCW from world +X (rtabmap Transform::theta()).
//
// Simplifying assumption: camera roll/pitch are small, so image-up roughly
// aligns with robot-forward (+X_base) and image-right with -Y_base. Floor
// arrows under significant camera tilt will incur a sub-bin error which is
// usually absorbed by the 45° quantization.
inline int imageAngleToWorldDirBin(float visualAngleDeg, float robotYawRad)
{
    // 180/π — defined locally so we don't depend on _USE_MATH_DEFINES.
    constexpr float kRadToDeg = 57.2957795130823208f;

    // image cw-from-up == base cw-from-forward (under the no-roll assumption).
    // World CCW yaw of arrow = robot yaw (CCW) - image angle (CW).
    const float worldCcwDeg = robotYawRad * kRadToDeg - visualAngleDeg;

    // Re-express as "clockwise from world north" for bin lookup.
    //   north (+Y) is CCW 90° from east (+X), so cwFromNorth = 90 - ccwFromEast.
    float cwFromNorth = 90.0f - worldCcwDeg;
    cwFromNorth = std::fmod(cwFromNorth, 360.0f);
    if (cwFromNorth < 0.0f) cwFromNorth += 360.0f;

    int idx = static_cast<int>(std::lround(cwFromNorth / 45.0f)) % 8;
    if (idx < 0) idx += 8;
    return idx + 1;  // 1..8
}

}  // namespace semantic

#endif /* SEMANTICBACKPROJECT_H_ */
