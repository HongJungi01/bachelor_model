"""
Single-call LLM runner — given one parking-lot image, return all relevant
detections in one round trip.

Each detection carries box (in image pixels), a category label, a confidence,
and (only for direction-bearing labels) a raw image-plane visual_angle in
[0, 360) measured clockwise from image-up. Camera-pose composition happens
downstream in the C++ SemanticMaskStore.

Provider is selected by the LLM_PROVIDER env var:
  unset / "gemini"     -> Gemini 3 Flash (default)
  "anthropic"          -> Claude Sonnet 4.6

The system prompt's category catalog is the single source of truth for what
gets tagged on the map. If you add a new label here, also extend
SemanticMaskStore::labelToGridCode() in C++.
"""

from __future__ import annotations

import base64
import io
import json
import os
from typing import Any

from PIL import Image
from pydantic import BaseModel


# ---------------------------------------------------------------------------
# Catalog & prompt
# ---------------------------------------------------------------------------

LABELS = (
    "parking_line",
    "lane_divider",
    "exit_sign",
    "pillar",
    "traffic_cone",
    "no_entry_sign",
    "construction_sign",
    "floor_arrow",
    "exit_area",
    "one_way_marker",
)

DIRECTIONAL_LABELS = (
    "floor_arrow",
    "one_way_marker",
    "exit_sign",
)


_SYSTEM_PROMPT = """\
You are a parking-lot scene analyzer for an autonomous-mapping pipeline.
Each request gives you ONE camera frame from an underground parking lot.
Your job is to find every relevant feature in the frame and return one
detection per feature, each with:

- a tight axis-aligned bounding box in IMAGE PIXEL coordinates, format
  [x1, y1, x2, y2] with (0, 0) at the top-left corner.
- one label from the fixed catalog below (no synonyms, no new categories).
- a confidence in [0, 1].
- visual_angle in [0, 360) for direction-bearing labels only, otherwise null.

Do not return boxes for cars, people, ceiling lamps, pipes, doors, drains,
trash, puddles, oil stains, or anything that is not in the catalog. Empty
detections list is a valid answer for a frame that contains nothing of
interest.

# Category catalog (use these exact label strings)

- "parking_line"
    A SOLID white line that runs PERPENDICULAR to the direction of vehicle
    travel and demarcates the edge of an individual parking stall. Typically
    appears in repeating pairs at constant spacing, bracketing a single
    vehicle's slot. Length is roughly the depth of one parked car. One box
    per visible stall edge.

- "lane_divider"
    A line that runs PARALLEL to the direction of vehicle travel and
    separates two driving lanes (or marks the centerline of a single
    aisle). May be DASHED (most common) or a single long solid line. DOES
    NOT bracket parking stalls. This is the #1 disambiguation target —
    earlier versions of this pipeline mislabeled center dividers as
    parking_line. If you see a dashed white line down the middle of an
    aisle, it is lane_divider, never parking_line.

- "exit_sign"
    A wall-, ceiling-, or beam-mounted illuminated EXIT marker (Korean:
    출구). Typically green or red text on a backlit panel, often with an
    arrow that points toward the exit. Includes ground-floor "EXIT THIS
    WAY" placards. The physical sign object only — see exit_area for the
    floor region.

- "pillar"
    A structural concrete column rising from floor to ceiling, often
    wrapped in yellow-and-black hazard tape near the base. Pillars block
    traversal and become walls in the occupancy grid.

- "traffic_cone"
    Orange/red traffic cone (Korean: 꼬깔/고깔) standing on the floor, used
    to block off areas, mark spills, or delimit construction. Generally
    < 1m tall, conical. One box per cone.

- "no_entry_sign"
    Red circular "DO NOT ENTER" / "진입금지" sign — either painted on the
    floor or mounted on a wall/post at the head of a one-way aisle. The
    agent must treat the area beyond it as a wall.

- "construction_sign"
    Yellow construction or maintenance warning sign / sandwich board /
    cordon. Anything indicating the area is not traversable due to active
    work.

- "floor_arrow"
    A directional arrow PAINTED ON THE FLOOR (single-headed for one-way
    aisles, sometimes double-headed for two-way). Indicates the locally-
    allowed direction of travel within a lane. Provide visual_angle equal
    to the direction the ARROW HEAD points toward, in image-plane degrees.

- "exit_area"
    A bounded REGION of floor (not a sign) leading toward a confirmed
    exit — i.e. a ramp, tunnel mouth, or apartment-tower pedestrian door.
    Use only when there is converging visual evidence (an adjacent
    exit_sign or floor_arrow points into it). Do NOT label random open
    floor as exit_area. This label has no visual_angle (return null).

- "one_way_marker"
    Sign or repeating arrow pattern that establishes a lane is one-way.
    Includes "DO NOT ENTER" wall signs at the head of a one-way aisle, and
    the repeated arrow stencils painted along a one-way aisle. Distinct
    from a single floor_arrow: one_way_marker carries the rule itself.
    Provide visual_angle equal to the allowed direction of travel.

# Disambiguation rules (in priority order)

1. parking_line vs lane_divider: judge by ORIENTATION of the line relative
   to the visible direction of travel of the nearest aisle. Perpendicular
   = parking_line. Parallel = lane_divider. Dashed = almost always
   lane_divider regardless of orientation.

2. exit_sign vs exit_area: exit_sign is the physical sign OBJECT;
   exit_area is the bounded floor REGION. They can both be present in the
   same scene but get different boxes with different labels.

3. floor_arrow vs one_way_marker: a single isolated arrow on the floor is
   floor_arrow. A repeating arrow pattern, OR a "DO NOT ENTER" sign with
   arrows together, is one_way_marker. When in doubt, prefer floor_arrow.

4. no_entry_sign vs construction_sign: red-circle "진입금지" is
   no_entry_sign. Yellow caution / 작업중 / maintenance sign is
   construction_sign. Both block traversal; the distinction matters for
   downstream rule application.

5. pillar vs other structures: pillars are concrete columns from floor to
   ceiling. Steel beams crossing the ceiling are NOT pillars (don't emit
   a box for those). A pillar wrapped in mirror or paint is still a
   pillar.

6. Tiny / blurry / occluded features: only emit a box if you are reasonably
   sure of the category. Honest omission is better than a confident wrong
   label. Use confidence < 0.5 to flag genuine uncertainty.

# visual_angle convention

For "floor_arrow", "one_way_marker", and "exit_sign" only — return the
direction in IMAGE-plane degrees, [0, 360), measured clockwise from
image-up:

  0   = pointing toward the TOP of the image
  90  = pointing toward the RIGHT
  180 = pointing toward the BOTTOM
  270 = pointing toward the LEFT
  Clockwise from 0.

This is a RAW visual angle as seen in the image. Do NOT compensate for
camera tilt, perspective, or robot heading — downstream SLAM has the
robot pose and camera extrinsics and will convert your raw angle into a
map-frame direction.

For "exit_sign" the angle should be the direction the sign's arrow points
(or the direction toward the exit relative to the sign's mount, if no
explicit arrow is drawn).

For ALL OTHER labels, return visual_angle as null.

# Confidence calibration

- 0.90 - 1.00   Unambiguous, clearly visible, fits a category exactly.
- 0.60 - 0.89   Visible but partially occluded, stylized, or at an
                awkward angle. Still a confident guess.
- 0.30 - 0.59   Inferred from shape and surrounding context; could
                plausibly be one of two labels.
- 0.00 - 0.29   Cannot really see it; usually better to omit the box.

# Output format

Return strictly the JSON shape requested by the response schema. Bounding
box coordinates are integers (or floats; either is fine) in image-pixel
space. Do not normalize. Do not return prose, markdown, or chain-of-
thought. Empty detections list is valid.
"""


class Detection(BaseModel):
    box: list[float]
    label: str
    confidence: float
    visual_angle: float | None = None


class DetectResult(BaseModel):
    detections: list[Detection]


# ---------------------------------------------------------------------------
# Provider implementations
# ---------------------------------------------------------------------------

def _pil_to_jpeg_bytes(img: Image.Image, quality: int = 85) -> bytes:
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


class _GeminiProvider:
    def __init__(self, model: str | None = None, api_key: str | None = None) -> None:
        from google import genai
        from google.genai import types

        self._types = types
        self._genai = genai
        self.client = genai.Client(api_key=api_key) if api_key else genai.Client()
        self.model = model or os.environ.get("GEMINI_MODEL", "gemini-3-flash")

    @property
    def name(self) -> str:
        return f"gemini ({self.model})"

    def detect(self, pil_image: Image.Image) -> DetectResult:
        types = self._types
        w, h = pil_image.size

        contents = [
            types.Part.from_bytes(
                data=_pil_to_jpeg_bytes(pil_image),
                mime_type="image/jpeg",
            ),
            types.Part.from_text(text=(
                f"Image is {w} pixels wide by {h} pixels tall. "
                f"Return all detections matching the catalog."
            )),
        ]

        response = self.client.models.generate_content(
            model=self.model,
            contents=contents,
            config=types.GenerateContentConfig(
                system_instruction=_SYSTEM_PROMPT,
                response_mime_type="application/json",
                response_schema=DetectResult,
            ),
        )

        parsed = getattr(response, "parsed", None)
        if isinstance(parsed, DetectResult):
            return parsed
        return DetectResult.model_validate_json(response.text)


class _AnthropicProvider:
    def __init__(self, model: str | None = None, api_key: str | None = None) -> None:
        import anthropic

        self.client = anthropic.Anthropic(api_key=api_key)
        self.model = model or os.environ.get("ANTHROPIC_MODEL", "claude-sonnet-4-6")

    @property
    def name(self) -> str:
        return f"anthropic ({self.model})"

    def detect(self, pil_image: Image.Image) -> DetectResult:
        w, h = pil_image.size
        b64 = base64.b64encode(_pil_to_jpeg_bytes(pil_image)).decode("ascii")

        schema = DetectResult.model_json_schema()

        response = self.client.messages.create(
            model=self.model,
            max_tokens=4096,
            system=[{
                "type": "text",
                "text": _SYSTEM_PROMPT,
                "cache_control": {"type": "ephemeral"},
            }],
            messages=[{
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/jpeg",
                            "data": b64,
                        },
                    },
                    {
                        "type": "text",
                        "text": (
                            f"Image is {w} pixels wide by {h} pixels tall. "
                            f"Return all detections matching the catalog."
                        ),
                    },
                ],
            }],
            output_config={
                "format": {"type": "json_schema", "schema": schema},
                "effort": "medium",
            },
        )
        text = next(b.text for b in response.content if b.type == "text")
        return DetectResult.model_validate(json.loads(text))


# ---------------------------------------------------------------------------
# Public façade
# ---------------------------------------------------------------------------

class LlmRunner:
    """Provider-agnostic single-call detector."""

    def __init__(self) -> None:
        provider = (os.environ.get("LLM_PROVIDER") or "gemini").lower()
        if provider == "anthropic":
            self._impl: Any = _AnthropicProvider()
        elif provider == "gemini":
            self._impl = _GeminiProvider()
        else:
            raise ValueError(
                f"Unknown LLM_PROVIDER={provider!r} (expected 'gemini' or 'anthropic')"
            )

    @property
    def name(self) -> str:
        return self._impl.name

    def detect(self, pil_image: Image.Image) -> DetectResult:
        result = self._impl.detect(pil_image)
        # Drop any boxes outside the catalog (defensive — well-behaved models
        # already obey the system prompt, but a stray label would break the
        # downstream code path).
        result.detections = [
            d for d in result.detections if d.label in LABELS
        ]
        return result


def maybe_create_runner() -> LlmRunner | None:
    """Return a runner if the selected provider's API key is set."""
    provider = (os.environ.get("LLM_PROVIDER") or "gemini").lower()
    if provider == "anthropic":
        if not os.environ.get("ANTHROPIC_API_KEY"):
            return None
    else:
        if not (os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")):
            return None
    return LlmRunner()
