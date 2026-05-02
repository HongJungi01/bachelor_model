"""
GroundingDINO runner via HuggingFace transformers.

Loads the model once and exposes detect(pil_image, prompt, ...).
The image passed in must already be at the resolution at which the
caller wants box coordinates returned — no resizing happens here.
"""

from __future__ import annotations

from typing import Any

import torch
from transformers import (
    AutoModelForZeroShotObjectDetection,
    AutoProcessor,
)


class GDinoRunner:
    def __init__(
        self,
        model_id: str = "IDEA-Research/grounding-dino-tiny",
        device: str | None = None,
    ) -> None:
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        self.model_id = model_id

        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = (
            AutoModelForZeroShotObjectDetection.from_pretrained(model_id)
            .to(device)
            .eval()
        )

    @torch.inference_mode()
    def detect(
        self,
        pil_image,
        prompt: str,
        box_threshold: float = 0.3,
        text_threshold: float = 0.25,
    ) -> dict[str, Any]:
        w, h = pil_image.size

        inputs = self.processor(
            images=pil_image, text=prompt, return_tensors="pt"
        ).to(self.device)

        outputs = self.model(**inputs)

        # target_sizes and threshold kwargs vary across transformers versions;
        # call with only the stable arguments and filter scores manually.
        try:
            raw = self.processor.post_process_grounded_object_detection(
                outputs,
                inputs.input_ids,
                target_sizes=[(h, w)],
            )[0]
        except TypeError:
            # older API without target_sizes — rescale boxes ourselves
            raw = self.processor.post_process_grounded_object_detection(
                outputs,
                inputs.input_ids,
            )[0]
            if len(raw["boxes"]):
                scale = torch.tensor([w, h, w, h], dtype=torch.float32,
                                     device=raw["boxes"].device)
                raw["boxes"] = raw["boxes"] * scale

        scores_t = raw["scores"]
        boxes_t  = raw["boxes"]
        labels_raw = raw["labels"]

        # Apply box_threshold; text_threshold applied via score (combined metric)
        keep = scores_t >= box_threshold
        boxes_t  = boxes_t[keep]
        scores_t = scores_t[keep]

        if isinstance(labels_raw, (list, tuple)):
            labels_filtered = [l for l, k in zip(labels_raw, keep.tolist()) if k]
        else:
            labels_filtered = labels_raw[keep]
            if hasattr(labels_filtered, "tolist"):
                labels_filtered = labels_filtered.tolist()

        return {
            "boxes":  boxes_t.cpu().tolist(),
            "scores": scores_t.cpu().tolist(),
            "labels": labels_filtered,
        }
