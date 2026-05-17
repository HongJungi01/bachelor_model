"""
FastAPI sidecar for parking-lot scene analysis.

Pipeline per /detect call:
  1. Decode the keyframe JPEG.
  2. Run YOLO11-seg in a single forward pass — returns boxes, labels,
     confidences, and per-instance masks.
  3. Union the masks into one binary PNG and return everything.

Endpoints:
  GET  /healthz       liveness + detector name
  POST /detect        image (jpeg b64) -> detections + union mask (PNG b64)
  GET  /debug_image   last processed frame with mask overlay + labeled boxes
  GET  /debug_json    last detection result as JSON (no image payload)

Boxes returned to the caller are in the *decoded image's* coordinate space
so the C++ caller can raster directly to the image resolution it sent.
"""

from __future__ import annotations

import base64
import io
import os
import threading
import time
import traceback
from contextlib import asynccontextmanager
from typing import Any

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import Response
from PIL import Image, ImageDraw, ImageFont
from pydantic import BaseModel, Field

from yolo_runner import YoloRunner

_yolo: YoloRunner | None = None
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
    global _yolo, _device

    weights = os.environ.get("YOLO_WEIGHTS", "models/best.pt")
    imgsz = int(os.environ.get("YOLO_IMGSZ", "1280"))
    conf = float(os.environ.get("YOLO_CONF", "0.25"))
    iou = float(os.environ.get("YOLO_IOU", "0.5"))

    _yolo = YoloRunner(weights=weights, imgsz=imgsz, conf=conf, iou=iou)
    _device = _yolo.device
    print(
        f"[semantic_service] YOLO loaded on device={_device} "
        f"(weights={_yolo.weights}, imgsz={imgsz}, conf={conf}, iou={iou})",
        flush=True,
    )
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
        description="Always null in YOLO pipeline; kept for C++ schema compatibility.",
    )


class DetectResponse(BaseModel):
    request_id: int | None
    width: int
    height: int
    inference_ms: int
    gdino_ms: int = 0   # legacy fields kept for log-grep continuity
    angle_ms: int = 0
    sam_ms: int = 0
    llm_ms: int = 0
    detections: list[Detection]
    mask_png_b64: str | None = None


@app.get("/healthz")
def healthz() -> dict[str, Any]:
    return {
        "status": "ok",
        "model_loaded": _yolo is not None,
        "device": _device,
        "detector": _yolo.name if _yolo is not None else None,
    }


@app.post("/detect", response_model=DetectResponse)
def detect(req: DetectRequest) -> DetectResponse:
    if _yolo is None:
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

    t0 = time.perf_counter()
    try:
        yolo_dets, mask_np = _yolo.detect(pil)
    except Exception as e:
        print(
            f"[semantic_service] /detect YOLO call FAILED: {type(e).__name__}: {e}",
            flush=True,
        )
        traceback.print_exc()
        raise HTTPException(status_code=502, detail=f"detector error: {e}") from e
    infer_ms = int((time.perf_counter() - t0) * 1000.0)

    detections = [
        Detection(
            box=d.box,
            label=d.label,
            confidence=d.confidence,
            visual_angle=None,
        )
        for d in yolo_dets
    ]

    print(
        f"[semantic_service] req={req.request_id}  "
        f"yolo={infer_ms}ms  det={len(detections)}",
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
            draw.rectangle([x1, y1 - 16, x1 + len(label_text) * 7, y1], fill=color)
            draw.text((x1 + 2, y1 - 15), label_text, fill=(255, 255, 255), font=font)

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
