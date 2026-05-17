"""
YOLO11-seg runner.

Loads a single ultralytics YOLO segmentation model and exposes detect() that
returns per-instance boxes/labels/scores plus a binary union mask covering
every detection. All outputs are in the input PIL image's pixel space — the
caller does not need to scale anything.

The label catalog comes from the loaded weights (model.names, sourced from
the dataset's data.yaml at train time). C++ resolveGridCode() decides which
of those label strings raster to wall — currently centerLine / parkingLine.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO


@dataclass
class Detection:
    box: list[float]      # [x1, y1, x2, y2] in input image pixels
    label: str
    confidence: float


class YoloRunner:
    def __init__(
        self,
        weights: str,
        device: str | None = None,
        imgsz: int = 1280,
        conf: float = 0.25,
        iou: float = 0.5,
    ) -> None:
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        self.weights = weights
        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        self.model = YOLO(weights)
        self.model.to(device)

    @property
    def name(self) -> str:
        return f"yolo ({self.weights})"

    @property
    def model_id(self) -> str:
        return self.weights

    @torch.inference_mode()
    def detect(
        self, pil_image: Image.Image
    ) -> tuple[list[Detection], np.ndarray]:
        w, h = pil_image.size
        result = self.model.predict(
            pil_image,
            imgsz=self.imgsz,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            verbose=False,
        )[0]

        detections: list[Detection] = []
        union = np.zeros((h, w), dtype=np.uint8)

        boxes = result.boxes
        masks = result.masks
        if boxes is None or len(boxes) == 0:
            return detections, union

        # Class index → label string comes straight from the loaded weights.
        names = result.names if isinstance(result.names, dict) else {}

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        clses = boxes.cls.cpu().numpy().astype(int)

        # Mask tensor shape: (N, mh, mw); resize to original (h, w) if needed.
        mask_arr: np.ndarray | None = None
        if masks is not None and masks.data is not None and len(masks.data) > 0:
            md = masks.data.cpu().numpy()  # (N, mh, mw) float 0/1
            if md.shape[1:] != (h, w):
                resized = np.zeros((md.shape[0], h, w), dtype=np.uint8)
                for i in range(md.shape[0]):
                    m_pil = Image.fromarray((md[i] > 0.5).astype(np.uint8) * 255)
                    m_pil = m_pil.resize((w, h), Image.NEAREST)
                    resized[i] = np.array(m_pil, dtype=np.uint8)
                mask_arr = resized
            else:
                mask_arr = (md > 0.5).astype(np.uint8) * 255

        for i in range(len(xyxy)):
            cls_idx = int(clses[i])
            label = names.get(cls_idx, f"class_{cls_idx}")

            x1, y1, x2, y2 = (float(v) for v in xyxy[i])
            x1 = max(0.0, min(float(w), x1))
            x2 = max(0.0, min(float(w), x2))
            y1 = max(0.0, min(float(h), y1))
            y2 = max(0.0, min(float(h), y2))
            if x2 - x1 < 1.0 or y2 - y1 < 1.0:
                continue

            detections.append(Detection(
                box=[x1, y1, x2, y2],
                label=label,
                confidence=float(confs[i]),
            ))

            if mask_arr is not None and i < mask_arr.shape[0]:
                union = np.maximum(union, mask_arr[i])

        return detections, union
