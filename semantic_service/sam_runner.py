"""
SAM (Segment Anything) runner via HuggingFace transformers.

Loads a single SamModel and exposes segment(pil_image, boxes) which returns
a binary union mask at the input image resolution. Box format:
[[x1, y1, x2, y2], ...] in image pixel coordinates of the same pil_image
that is passed in.

The SAM image processor handles internal resizing to 1024 and rescales the
output mask back to the original image size, so callers can pass any
resolution.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np
import torch
from PIL import Image
from transformers import SamModel, SamProcessor


class SamRunner:
    def __init__(
        self,
        model_id: str = "facebook/sam-vit-base",
        device: str | None = None,
    ) -> None:
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        self.model_id = model_id

        self.processor = SamProcessor.from_pretrained(model_id)
        self.model = SamModel.from_pretrained(model_id).to(device).eval()

    @torch.inference_mode()
    def segment(
        self,
        pil_image: Image.Image,
        boxes: Sequence[Sequence[float]],
    ) -> np.ndarray:
        """
        Returns a binary union mask (H x W, uint8 0/255) over all boxes.
        Empty boxes list -> returns an all-zero mask of the input size.
        """
        w, h = pil_image.size
        if not boxes:
            return np.zeros((h, w), dtype=np.uint8)

        # SAM expects input_boxes shape (batch=1, num_boxes, 4).
        input_boxes = [[list(map(float, b)) for b in boxes]]

        inputs = self.processor(
            pil_image,
            input_boxes=input_boxes,
            return_tensors="pt",
        ).to(self.device)

        outputs = self.model(**inputs, multimask_output=False)

        # post_process_masks: list[tensor], one per image.
        # With multimask_output=False, tensor shape is
        # (num_boxes, 1, H, W) bool, already at original image size.
        masks = self.processor.image_processor.post_process_masks(
            outputs.pred_masks.cpu(),
            inputs["original_sizes"].cpu(),
            inputs["reshaped_input_sizes"].cpu(),
        )[0]

        np_masks = masks.numpy()
        union = np.any(np_masks, axis=(0, 1))
        return (union.astype(np.uint8) * 255)
