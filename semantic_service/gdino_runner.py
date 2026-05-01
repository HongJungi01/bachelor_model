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

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            target_sizes=[(h, w)],
        )[0]

        boxes = results["boxes"].cpu().tolist()
        scores = results["scores"].cpu().tolist()
        labels = results["labels"]
        if hasattr(labels, "cpu"):
            labels = labels.cpu().tolist()

        return {"boxes": boxes, "scores": scores, "labels": labels}
