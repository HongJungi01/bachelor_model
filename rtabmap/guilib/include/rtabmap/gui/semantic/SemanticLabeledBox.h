/*
 * SemanticLabeledBox.h
 *
 * Shared POD shared between SemanticWorker (which produces it from the
 * /classify_batch response) and SemanticMaskStore (which consumes it to
 * rasterise per-keyframe semantic masks).
 *
 * Kept in its own tiny header so SemanticMaskStore does not have to
 * transitively include httplib / nlohmann_json from SemanticWorker.h.
 */

#ifndef SEMANTICLABELEDBOX_H_
#define SEMANTICLABELEDBOX_H_

#include <string>

namespace semantic {

struct LabeledBox {
    float       x1 = 0, y1 = 0, x2 = 0, y2 = 0;  // image pixels
    std::string label;                             // Gemini label, may be empty
    float       confidence  = 0.0f;
    // Image-plane angle in [0, 360), -1 if not applicable. Only emitted by
    // Gemini for direction labels (floor_arrow, one_way_marker, lane_divider).
    // Downstream consumers combine this with robot pose + camera extrinsics
    // to derive a map-frame direction.
    float       visualAngle = -1.0f;
};

}  // namespace semantic

#endif /* SEMANTICLABELEDBOX_H_ */
