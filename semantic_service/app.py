"""
FastAPI sidecar for GroundingDINO open-vocabulary detection.

Endpoints:
  GET  /healthz   liveness + device probe (200 even before model loaded)
  POST /detect    image (jpeg b64) + prompt -> boxes (original-image coords)

Boxes are returned in the *decoded image's* coordinate space — i.e. the
resolution after JPEG decode but BEFORE any downscale done internally for
inference. The C++ caller can therefore raster directly to the image
resolution it sent.
"""

from __future__ import annotations

import base64
import io
import time
from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, HTTPException
from PIL import Image
from pydantic import BaseModel, Field

from gdino_runner import GDinoRunner

# Long edge after internal resize — keeps inference latency bounded.
INFERENCE_MAX_EDGE = 800

_runner: GDinoRunner | None = None
_device: str = "unknown"


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _runner, _device
    _runner = GDinoRunner()
    _device = _runner.device
    print(f"[semantic_service] model loaded on device={_device}", flush=True)
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
        "model_loaded": _runner is not None,
        "device": _device,
    }


@app.post("/detect", response_model=DetectResponse)
def detect(req: DetectRequest) -> DetectResponse:
    if _runner is None:
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
    infer_ms = int((time.perf_counter() - t0) * 1000.0)

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

    return DetectResponse(
        request_id=req.request_id,
        width=orig_w,
        height=orig_h,
        inference_ms=infer_ms,
        detections=detections,
        mask_png_b64=None,
    )
