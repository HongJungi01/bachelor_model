"""
FastAPI sidecar for Grounded-SAM open-vocabulary segmentation.

Pipeline: GroundingDINO produces text-prompted boxes; SAM converts those
boxes into pixel-accurate masks; the union mask is returned as a PNG.

Endpoints:
  GET  /healthz       liveness + device probe (200 even before model loaded)
  POST /detect        image (jpeg b64) + prompt -> boxes + union mask (PNG b64)
  GET  /debug_image   last processed frame with mask overlay + boxes (JPEG)
  GET  /debug_json    last detection result as JSON (no image payload)

Boxes and the mask are both in the *decoded image's* coordinate space — i.e.
the resolution after JPEG decode but BEFORE any downscale done internally
for inference. The C++ caller can therefore raster directly to the image
resolution it sent.
"""

from __future__ import annotations

import base64
import io
import threading
import time
from contextlib import asynccontextmanager
from typing import Any

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import Response
from PIL import Image, ImageDraw, ImageFont
from pydantic import BaseModel, Field

from gdino_runner import GDinoRunner
from sam_runner import SamRunner

# Long edge after internal resize — keeps inference latency bounded.
INFERENCE_MAX_EDGE = 800

_runner: GDinoRunner | None = None
_sam: SamRunner | None = None
_device: str = "unknown"

# Last-result store — updated under _debug_lock after each /detect call.
_debug_lock = threading.Lock()
_debug_jpeg: bytes | None = None          # annotated frame as JPEG bytes
_debug_meta: dict[str, Any] | None = None # last detection summary

_COLORS = [
    (255,  80,  80),
    ( 80, 200,  80),
    ( 80, 120, 255),
    (255, 200,  50),
    (200,  80, 255),
]


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _runner, _sam, _device
    _runner = GDinoRunner()
    _sam = SamRunner(device=_runner.device)
    _device = _runner.device
    print(
        f"[semantic_service] models loaded on device={_device} "
        f"(gdino={_runner.model_id}, sam={_sam.model_id})",
        flush=True,
    )
    yield


app = FastAPI(title="semantic_service", lifespan=lifespan)


class DetectRequest(BaseModel):
    image_jpeg_b64: str
    prompt: str = "parking space line . lane marking"
    box_threshold: float = 0.3
    text_threshold: float = 0.25
    request_id: int | None = None


class Detection(BaseModel):
    box: list[float] = Field(..., description="[x1,y1,x2,y2] in original image coords")
    score: float
    label: str


class DetectResponse(BaseModel):
    request_id: int | None
    width: int
    height: int
    inference_ms: int
    detections: list[Detection]
    mask_png_b64: str | None = None


@app.get("/healthz")
def healthz() -> dict[str, Any]:
    return {
        "status": "ok",
        "model_loaded": _runner is not None and _sam is not None,
        "device": _device,
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

    long_edge = max(orig_w, orig_h)
    if long_edge > INFERENCE_MAX_EDGE:
        scale = INFERENCE_MAX_EDGE / float(long_edge)
        new_w = int(round(orig_w * scale))
        new_h = int(round(orig_h * scale))
        infer_pil = pil.resize((new_w, new_h), Image.BILINEAR)
        sx = orig_w / float(new_w)
        sy = orig_h / float(new_h)
    else:
        infer_pil = pil
        sx = sy = 1.0

    t0 = time.perf_counter()
    out = _runner.detect(
        infer_pil,
        prompt=req.prompt,
        box_threshold=req.box_threshold,
        text_threshold=req.text_threshold,
    )

    detections: list[Detection] = []
    for box, score, label in zip(out["boxes"], out["scores"], out["labels"]):
        x1, y1, x2, y2 = box
        x1 *= sx
        x2 *= sx
        y1 *= sy
        y2 *= sy
        x1 = max(0.0, min(float(orig_w), x1))
        x2 = max(0.0, min(float(orig_w), x2))
        y1 = max(0.0, min(float(orig_h), y1))
        y2 = max(0.0, min(float(orig_h), y2))
        if x2 <= x1 or y2 <= y1:
            continue
        detections.append(
            Detection(
                box=[x1, y1, x2, y2],
                score=float(score),
                label=str(label),
            )
        )

    # SAM: run on the already-downscaled infer_pil (GDINO's resolution) so SAM
    # doesn't need to re-encode a full-res image. Boxes are scaled down to
    # infer_pil coords; the returned mask is scaled back to original size.
    sam_boxes = [[x / sx, y / sy, x2 / sx, y2 / sy]
                 for x, y, x2, y2 in (d.box for d in detections)]
    mask_small = _sam.segment(infer_pil, sam_boxes)
    if mask_small.shape != (orig_h, orig_w):
        mask_np = np.array(
            Image.fromarray(mask_small).resize((orig_w, orig_h), Image.NEAREST)
        )
    else:
        mask_np = mask_small

    infer_ms = int((time.perf_counter() - t0) * 1000.0)

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

        # Tint mask pixels green so users can see SAM's true segmentation
        # against the GDINO box outlines.
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
            label_text = f"{det.label} {det.score:.2f}"
            draw.rectangle([x1, y1 - 16, x1 + len(label_text) * 7, y1], fill=color)
            draw.text((x1 + 2, y1 - 15), label_text, fill=(255, 255, 255), font=font)

        mask_pixels = int(mask_np.sum() // 255) if mask_np is not None else 0
        info = (
            f"req={request_id}  {infer_ms}ms  "
            f"{len(detections)} det  {mask_pixels}px mask  "
            f"{orig_pil.width}x{orig_pil.height}"
        )
        draw.rectangle([0, 0, len(info) * 7 + 4, 16], fill=(0, 0, 0, 160))
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
                    {"box": d.box, "score": d.score, "label": d.label}
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
