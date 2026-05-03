"""
Gemini 3.1 Flash Lite (preview) classifier for L3.

Same interface as the Claude runner — given (image, pre-detected boxes)
tuples for up to 5 frames per call, returns a class label + confidence
per box. Box coordinates are NOT generated here; they come from L1+L2.
Gemini's job is purely to attach semantic labels to ids.

Why Gemini Flash Lite for this:
- Vision input is native and cheap (preview pricing well below Sonnet).
- Structured output via response_schema produces parseable JSON every
  time when paired with a Pydantic model.
- Implicit context caching (Gemini 2.5+) reuses the system prompt across
  repeated calls automatically — no explicit cache object needed for
  the typical hot-loop case. If the preview tier's caching behavior
  differs, the code still works (cached token count just reads zero).

If Google revises the model ID with a dated suffix (e.g.
"gemini-3.1-flash-lite-preview-MM-DD"), update _MODEL below.
"""

from __future__ import annotations

import io
import json
import os
from typing import Any

from google import genai
from google.genai import types
from PIL import Image
from pydantic import BaseModel


_MODEL = "gemini-3.1-flash-lite-preview"


_SYSTEM_PROMPT = """\
You are a parking-lot scene classifier for an autonomous-mapping
pipeline. Your input on every request is up to 5 camera frames from an
underground parking lot. Each frame ships as a JPEG image followed by a
text block listing pre-computed bounding boxes (each with an integer
id and [x1, y1, x2, y2] coordinates in the image's pixel space). Those
boxes were produced by an upstream detector (a mix of classical CV and
Florence-2). Coordinates are CORRECT — do not second-guess them, do
not invent new boxes, do not merge ids, and do not omit ids. For every
id you receive, return exactly one classification.

# Category catalog (use these exact label strings — no variants)

- "parking_line"
    A SOLID white line that runs PERPENDICULAR to the direction of
    vehicle travel and demarcates the edge of an individual parking
    stall. Typically appears in repeating pairs at constant spacing,
    bracketing a single vehicle's slot. Length is roughly the depth
    of one parked car.

- "lane_divider"
    A line that runs PARALLEL to the direction of vehicle travel and
    separates two driving lanes (or marks the centerline of a single
    aisle). May be DASHED (most common) or a single long solid line.
    DOES NOT bracket parking stalls. This is the #1 disambiguation
    target — earlier versions of this pipeline mislabeled center
    dividers as parking_line. If you see a dashed white line down
    the middle of an aisle, it is lane_divider, never parking_line.

- "exit_sign"
    A wall-, ceiling-, or beam-mounted illuminated EXIT marker
    (Korean: 출구). Typically green or red text on a backlit panel.
    Includes ground-floor "EXIT THIS WAY" placards. The physical
    sign object only — see exit_area for the floor region.

- "pillar"
    A structural concrete column rising from floor to ceiling, often
    wrapped in yellow-and-black hazard tape near the base. Pillars
    block traversal and become walls in the occupancy grid.

- "traffic_cone"
    Orange/red traffic cone (Korean: 꼬깔/고깔) standing on the
    floor, used to block off areas, mark spills, or delimit
    construction. Generally < 1m tall, conical.

- "no_entry_sign"
    Red circular "DO NOT ENTER" / "진입금지" sign — either painted
    on the floor or mounted on a wall/post at the head of a one-way
    aisle. The agent must treat the area beyond it as a wall.

- "construction_sign"
    Yellow construction or maintenance warning sign / sandwich
    board / cordon. Anything indicating the area is not
    traversable due to active work.

- "floor_arrow"
    A directional arrow PAINTED ON THE FLOOR (single-headed for
    one-way aisles, sometimes double-headed for two-way). Indicates
    the locally-allowed direction of travel within a lane. The
    agent uses these to derive lane direction codes.

- "exit_area"
    A bounded REGION of floor (not a sign) leading toward a
    confirmed exit — i.e. a ramp, tunnel mouth, or apartment-tower
    pedestrian door. Use only when there is converging visual
    evidence (an adjacent exit_sign or floor_arrow points into it).
    Do NOT label random open floor as exit_area.

- "one_way_marker"
    Sign or repeating arrow pattern that establishes a lane is
    one-way. Includes "DO NOT ENTER" wall signs at the head of a
    one-way aisle. Distinct from a single floor_arrow: one_way_
    marker carries the rule, floor_arrow merely indicates direction.

- "intersection"
    The open floor region where two driving aisles meet. Typically
    bounded by parking_line ranges on three sides and an arrow or
    sign on the fourth. Used to anchor the topological graph.

- "other"
    The box does not fit any category above. Vehicles, parked cars,
    people, ceiling lamps, pipes, drains, doors, trash, puddles,
    tire marks, oil stains — all "other". Do not invent new
    categories. If you cannot identify the object at all because
    the box is tiny, blurry, or out of view, also use "other" with
    confidence <= 0.3.

# Critical disambiguation rules

1. parking_line vs lane_divider: judge by ORIENTATION of the line
   relative to the visible direction of travel of the nearest aisle.
   Perpendicular = parking_line. Parallel = lane_divider. Dashed =
   almost always lane_divider regardless of orientation.

2. exit_sign vs exit_area: exit_sign is the physical sign OBJECT;
   exit_area is the bounded floor REGION. They can both be present
   in the same scene but get different labels on different boxes.

3. floor_arrow vs one_way_marker: a single isolated arrow on the
   floor is floor_arrow. A repeating arrow pattern, OR a "DO NOT
   ENTER" sign + arrows together, is one_way_marker. When in doubt,
   prefer floor_arrow.

4. no_entry_sign vs construction_sign: red-circle "진입금지" is
   no_entry_sign. Yellow caution / 작업중 / maintenance sign is
   construction_sign. Both block traversal; the distinction matters
   for downstream rule application.

5. pillar vs other: pillars are concrete columns from floor to
   ceiling. Steel beams crossing the ceiling are NOT pillars
   (label "other"). A pillar wrapped in mirror or paint is still
   a pillar.

6. Small / tiny / blurry boxes: if you genuinely cannot see what
   is inside the box, label "other" with confidence <= 0.3 rather
   than guessing. Honest uncertainty is more useful downstream
   than a confident wrong label.

7. Same scene across frames: the same physical pillar may appear
   in 4 of the 5 frames with different ids. Label each
   independently — do not try to track identity across frames.

# Visual angle (only for direction labels)

For three labels — and ONLY these three — also return visual_angle:

- floor_arrow      direction the ARROW HEAD points toward.
- one_way_marker   allowed direction of travel through this marker.
- lane_divider     direction the line RUNS (parallel to traffic flow).
                   For a vertical line in the image, return 0 or 180
                   (either end is acceptable; downstream takes mod 180).

Convention: degrees in [0, 360), measured in the IMAGE plane.
  0   = pointing toward the TOP of the image
  90  = pointing toward the RIGHT
  180 = pointing toward the BOTTOM
  270 = pointing toward the LEFT
  Clockwise from 0.

This is a RAW visual angle as seen in the image — do NOT attempt to
compensate for camera tilt, perspective, or robot heading. Downstream
SLAM has the robot pose and camera extrinsics; it converts your raw
angle into a map-frame direction.

For ALL OTHER labels, return visual_angle as null. Do not invent angles
for pillars, signs, parking lines, etc.

# Confidence calibration

Return a confidence in [0, 1]. Calibrate honestly — downstream uses
this to weight the classification:

- 0.90 - 1.00   Unambiguous, clearly visible, fits a category exactly.
- 0.60 - 0.89   Visible but partially occluded, stylized, or at
                an awkward angle. Still a confident guess.
- 0.30 - 0.59   Inferred from shape and surrounding context;
                could plausibly be one of two labels.
- 0.00 - 0.29   Cannot really see it; pair with label "other".

Avoid systematic over-confidence. A reviewer looking at the image
should agree with your confidence estimate.

# Output format

Return strictly the JSON shape requested by the response schema.
For each frame, list classifications matching the input box ids.
Order does not matter to the consumer (it joins by id) but stable
order helps debugging — emit ids in the same order you received
them. Every input id must appear exactly once in the output.
"""


class _Classification(BaseModel):
    id: int
    label: str
    confidence: float
    # Image-plane angle for direction labels only — see system prompt.
    # null for all other labels.
    visual_angle: float | None = None


class _FrameResult(BaseModel):
    request_id: int
    classifications: list[_Classification]


class _BatchResult(BaseModel):
    frames: list[_FrameResult]


def _pil_to_jpeg_bytes(img: Image.Image, quality: int = 85) -> bytes:
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


class GeminiClassifier:
    """L3 semantic classifier backed by Gemini 3.1 Flash Lite (preview)."""

    def __init__(self, model: str = _MODEL, api_key: str | None = None) -> None:
        # SDK reads GEMINI_API_KEY / GOOGLE_API_KEY from env when api_key=None.
        self.client = (
            genai.Client(api_key=api_key) if api_key else genai.Client()
        )
        self.model = model

    def classify_batch(self, frames: list[dict[str, Any]]) -> dict[str, Any]:
        """
        frames: up to 5 entries, each:
          {
            "request_id": int,
            "image":       PIL.Image,
            "boxes":       [{"id": int, "box": [x1, y1, x2, y2]}, ...]
          }

        Returns:
          {
            "result": {"frames": [{"request_id": int, "classifications": [...]}, ...]},
            "usage":  {prompt_tokens, cached_tokens, candidates_tokens, total_tokens}
          }
        """
        if not frames:
            raise ValueError("classify_batch requires at least one frame")
        if len(frames) > 5:
            raise ValueError("classify_batch accepts at most 5 frames per call")

        contents: list[types.Part] = []
        for f in frames:
            contents.append(types.Part.from_bytes(
                data=_pil_to_jpeg_bytes(f["image"]),
                mime_type="image/jpeg",
            ))
            box_lines = [
                f"  id={b['id']}: [{int(b['box'][0])}, {int(b['box'][1])}, "
                f"{int(b['box'][2])}, {int(b['box'][3])}]"
                for b in f["boxes"]
            ]
            contents.append(types.Part.from_text(text=(
                f"Frame request_id={f['request_id']}, "
                f"{len(f['boxes'])} box(es) "
                f"(format — id: [x1, y1, x2, y2] in image pixels):\n"
                + ("\n".join(box_lines) if box_lines else "  (none)")
            )))

        response = self.client.models.generate_content(
            model=self.model,
            contents=contents,
            config=types.GenerateContentConfig(
                system_instruction=_SYSTEM_PROMPT,
                response_mime_type="application/json",
                response_schema=_BatchResult,
            ),
        )

        # response.parsed is the auto-parsed Pydantic model when
        # response_schema is set; fall back to manual JSON parse if the
        # SDK couldn't materialize it (older SDK builds, edge cases).
        parsed = getattr(response, "parsed", None)
        if isinstance(parsed, _BatchResult):
            result_dict = parsed.model_dump()
        else:
            result_dict = json.loads(response.text)

        usage = getattr(response, "usage_metadata", None)
        usage_dict = {
            "prompt_tokens":     getattr(usage, "prompt_token_count", 0) if usage else 0,
            "cached_tokens":     getattr(usage, "cached_content_token_count", 0) if usage else 0,
            "candidates_tokens": getattr(usage, "candidates_token_count", 0) if usage else 0,
            "total_tokens":      getattr(usage, "total_token_count", 0) if usage else 0,
        }
        return {"result": result_dict, "usage": usage_dict}


def maybe_create_classifier() -> GeminiClassifier | None:
    """Return a classifier if a Gemini/Google API key env var is set."""
    if not (os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")):
        return None
    return GeminiClassifier()
