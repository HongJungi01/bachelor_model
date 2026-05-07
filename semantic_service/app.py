"""
FastAPI sidecar for single-call LLM-based parking-lot scene analysis.

Pipeline per /detect call:
  1. Decode the keyframe JPEG.
  2. Hand it to the LLM runner (Gemini 3 Flash by default; Claude Sonnet 4.6
     when LLM_PROVIDER=anthropic). The LLM returns boxes + labels +
     confidences, plus an image-plane visual_angle for direction-bearing
     labels.
  3. Run SAM on those boxes to refine into a pixel-accurate union mask.
  4. Return detections (with label/visual_angle) and the mask PNG.

Endpoints:
  GET  /healthz       liveness + provider name
  POST /detect        image (jpeg b64) -> detections + union mask (PNG b64)
  GET  /debug_image   last processed frame with mask overlay + labeled boxes
  GET  /debug_json    last detection result as JSON (no image payload)

Boxes returned to the caller are in the *decoded image's* coordinate space
(i.e. the resolution after JPEG decode). The C++ caller can therefore
raster directly to the image resolution it sent.
"""

from __future__ import annotations

import base64
import io
import math
import os
import threading
import time
from contextlib import asynccontextmanager
from typing import Any

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import Response
from PIL import Image, ImageDraw, ImageFont
from pydantic import BaseModel, Field

from llm_runner import DIRECTIONAL_LABELS, LlmRunner, maybe_create_runner
from sam_runner import SamRunner

# Long edge limits for the two stages.
# LLM: 1280px keeps upload + processing under ~3s on Gemini Flash.
#   Boxes are returned in LLM-image coords and scaled back to original.
# SAM: 800px bounds local GPU latency.
LLM_MAX_EDGE = 1280
SAM_MAX_EDGE = 800

_runner: LlmRunner | None = None
_sam: SamRunner | None = None
_device: str = "unknown"

_debug_lock = threading.Lock()
_debug_jpeg: bytes | None = None
_debug_meta: dict[str, Any] | None = None

_COLORS = [
    (255,  80,  80),
    ( 80, 200,  80),
    ( 80, 120, 255),
    (255, 200,  50),
    (200,  80, 255),
    ( 80, 220, 220),
    (255, 140,  60),
]


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _runner, _sam, _device

    _sam = SamRunner()
    _device = _sam.device
    print(
        f"[semantic_service] SAM loaded on device={_device} (model={_sam.model_id})",
        flush=True,
    )

    _runner = maybe_create_runner()
    if _runner is None:
        provider = (os.environ.get("LLM_PROVIDER") or "gemini").lower()
        env = "ANTHROPIC_API_KEY" if provider == "anthropic" else "GEMINI_API_KEY"
        print(
            f"[semantic_service] LLM disabled — set {env} (LLM_PROVIDER={provider}) "
            f"to enable /detect",
            flush=True,
        )
    else:
        print(f"[semantic_service] LLM ready: {_runner.name}", flush=True)
    yield


app = FastAPI(title="semantic_service", lifespan=lifespan)


class DetectRequest(BaseModel):
    image_jpeg_b64: str
    request_id: int | None = None


class Detection(BaseModel):
    box: list[float] = Field(..., description="[x1, y1, x2, y2] in original image pixels")
    label: str
    confidence: float
    visual_angle: float | None = Field(
        None,
        description=(
            "Image-plane angle [0, 360), clockwise from image-up. "
            "Only set for floor_arrow / one_way_marker / exit_sign."
        ),
    )


class DetectResponse(BaseModel):
    request_id: int | None
    width: int
    height: int
    inference_ms: int
    llm_ms: int = 0
    sam_ms: int = 0
    detections: list[Detection]
    mask_png_b64: str | None = None


@app.get("/healthz")
def healthz() -> dict[str, Any]:
    return {
        "status": "ok",
        "model_loaded": _runner is not None and _sam is not None,
        "device": _device,
        "provider": _runner.name if _runner is not None else None,
    }


@app.post("/detect", response_model=DetectResponse)
def detect(req: DetectRequest) -> DetectResponse:
    if _runner is None or _sam is None:
        raise HTTPException(status_code=503, detail="model not loaded yet")

    try:
        raw = base64.b64decode(req.image_jpeg_b64, validate=False)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"bad base64: {e}") from e

    try:
        pil = Image.open(io.BytesIO(raw)).convert("RGB")
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"bad jpeg: {e}") from e

    orig_w, orig_h = pil.size

    # Downscale for LLM to reduce upload + API processing time.
    llm_long = max(orig_w, orig_h)
    if llm_long > LLM_MAX_EDGE:
        llm_scale = LLM_MAX_EDGE / float(llm_long)
        llm_pil = pil.resize(
            (int(round(orig_w * llm_scale)), int(round(orig_h * llm_scale))),
            Image.BILINEAR,
        )
    else:
        llm_pil = pil
        llm_scale = 1.0

    t0 = time.perf_counter()

    try:
        llm_result = _runner.detect(llm_pil)
    except Exception as e:
        import traceback
        print(
            f"[semantic_service] /detect LLM call FAILED: {type(e).__name__}: {e}",
            flush=True,
        )
        traceback.print_exc()
        raise HTTPException(status_code=502, detail=f"llm error: {e}") from e

    t_llm = time.perf_counter()
    llm_ms = int((t_llm - t0) * 1000.0)

    detections: list[Detection] = []
    for d in llm_result.detections:
        if len(d.box) != 4:
            continue
        # Scale boxes from LLM-image coords back to original resolution.
        x1, y1, x2, y2 = (float(v) / llm_scale for v in d.box)
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1
        x1 = max(0.0, min(float(orig_w), x1))
        x2 = max(0.0, min(float(orig_w), x2))
        y1 = max(0.0, min(float(orig_h), y1))
        y2 = max(0.0, min(float(orig_h), y2))
        if x2 - x1 < 1.0 or y2 - y1 < 1.0:
            continue
        va = d.visual_angle
        if d.label not in DIRECTIONAL_LABELS:
            va = None
        elif va is not None:
            va = float(va) % 360.0
        detections.append(
            Detection(
                box=[x1, y1, x2, y2],
                label=d.label,
                confidence=float(d.confidence),
                visual_angle=va,
            )
        )

    # SAM refinement on a downscaled copy to bound latency. Boxes scale
    # the same way; the mask is upsampled back to the original resolution.
    long_edge = max(orig_w, orig_h)
    if long_edge > SAM_MAX_EDGE:
        scale = SAM_MAX_EDGE / float(long_edge)
        sam_w = int(round(orig_w * scale))
        sam_h = int(round(orig_h * scale))
        sam_pil = pil.resize((sam_w, sam_h), Image.BILINEAR)
        sx = orig_w / float(sam_w)
        sy = orig_h / float(sam_h)
    else:
        sam_pil = pil
        sx = sy = 1.0

    sam_boxes = [
        [d.box[0] / sx, d.box[1] / sy, d.box[2] / sx, d.box[3] / sy]
        for d in detections
    ]
    t_sam0 = time.perf_counter()
    mask_small = _sam.segment(sam_pil, sam_boxes)
    if mask_small.shape != (orig_h, orig_w):
        mask_np = np.array(
            Image.fromarray(mask_small).resize((orig_w, orig_h), Image.NEAREST)
        )
    else:
        mask_np = mask_small

    sam_ms = int((time.perf_counter() - t_sam0) * 1000.0)
    infer_ms = int((time.perf_counter() - t0) * 1000.0)
    print(
        f"[semantic_service] req={req.request_id}  "
        f"total={infer_ms}ms  llm={llm_ms}ms  sam={sam_ms}ms  "
        f"det={len(detections)}",
        flush=True,
    )

    mask_b64: str | None = None
    if detections and int(mask_np.any()):
        mask_pil = Image.fromarray(mask_np, mode="L")
        mbuf = io.BytesIO()
        mask_pil.save(mbuf, format="PNG", compress_level=1)
        mask_b64 = base64.b64encode(mbuf.getvalue()).decode("ascii")

    resp = DetectResponse(
        request_id=req.request_id,
        width=orig_w,
        height=orig_h,
        inference_ms=infer_ms,
        llm_ms=llm_ms,
        sam_ms=sam_ms,
        detections=detections,
        mask_png_b64=mask_b64,
    )

    threading.Thread(
        target=_update_debug,
        args=(pil, detections, mask_np, infer_ms, req.request_id),
        daemon=True,
    ).start()

    return resp


def _update_debug(
    orig_pil: Image.Image,
    detections: list[Detection],
    mask_np: np.ndarray,
    infer_ms: int,
    request_id: int | None,
) -> None:
    try:
        img = orig_pil.copy()

        if mask_np is not None and mask_np.any():
            base = np.array(img, dtype=np.uint8)
            sel = mask_np > 0
            base[sel] = (
                base[sel].astype(np.uint16) * np.array([1, 3, 1], dtype=np.uint16) // 4
            ).astype(np.uint8)
            base[sel, 1] = np.maximum(base[sel, 1], 180)
            img = Image.fromarray(base)

        draw = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype("arial.ttf", max(12, orig_pil.width // 60))
        except Exception:
            font = ImageFont.load_default()

        for i, det in enumerate(detections):
            color = _COLORS[i % len(_COLORS)]
            x1, y1, x2, y2 = (int(v) for v in det.box)
            draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
            label_text = f"{det.label} {det.confidence:.2f}"
            if det.visual_angle is not None:
                label_text += f" @{det.visual_angle:.0f}°"
            draw.rectangle([x1, y1 - 16, x1 + len(label_text) * 7, y1], fill=color)
            draw.text((x1 + 2, y1 - 15), label_text, fill=(255, 255, 255), font=font)

            # Draw an angle arrow inside the box for directional labels.
            # 0° = up; 90° = right; clockwise. In image XY:
            #   dx =  sin(theta), dy = -cos(theta)
            if det.visual_angle is not None:
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                length = max(8.0, min(x2 - x1, y2 - y1) * 0.4)
                theta = math.radians(det.visual_angle)
                dx = math.sin(theta) * length
                dy = -math.cos(theta) * length
                draw.line(
                    [(cx, cy), (cx + dx, cy + dy)],
                    fill=color,
                    width=3,
                )

        mask_pixels = int(mask_np.sum() // 255) if mask_np is not None else 0
        info = (
            f"req={request_id}  {infer_ms}ms  "
            f"{len(detections)} det  {mask_pixels}px mask  "
            f"{orig_pil.width}x{orig_pil.height}"
        )
        draw.rectangle([0, 0, len(info) * 7 + 4, 16], fill=(0, 0, 0))
        draw.text((2, 1), info, fill=(255, 255, 0), font=font)

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        jpeg_bytes = buf.getvalue()

        with _debug_lock:
            global _debug_jpeg, _debug_meta
            _debug_jpeg = jpeg_bytes
            _debug_meta = {
                "request_id": request_id,
                "inference_ms": infer_ms,
                "width": orig_pil.width,
                "height": orig_pil.height,
                "detections": [
                    {
                        "box": d.box,
                        "label": d.label,
                        "confidence": d.confidence,
                        "visual_angle": d.visual_angle,
                    }
                    for d in detections
                ],
            }
    except Exception as e:
        print(f"[semantic_service] debug render error: {e}", flush=True)


@app.get("/debug_image", responses={200: {"content": {"image/jpeg": {}}}})
def debug_image() -> Response:
    with _debug_lock:
        data = _debug_jpeg
    if data is None:
        raise HTTPException(status_code=404, detail="no frame processed yet")
    return Response(content=data, media_type="image/jpeg")


@app.get("/debug_json")
def debug_json() -> Any:
    with _debug_lock:
        meta = _debug_meta
    if meta is None:
        raise HTTPException(status_code=404, detail="no frame processed yet")
    return meta
