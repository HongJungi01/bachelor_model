"""
Florence-2 phrase-grounding runner via HuggingFace transformers.

Loads Microsoft Florence-2 once and exposes detect(pil_image, prompt, ...)
with the same return shape as GDinoRunner so the rest of the service
(SAM mask conversion, debug renderer) is unchanged.

The prompt string is treated as the *caption* for Florence-2's
<CAPTION_TO_PHRASE_GROUNDING> task: every noun phrase in the caption is
grounded to one or more boxes labeled with that phrase. Use commas to
separate categories, e.g.
    "parking stall lines, exit signs, pillars, traffic cones"

Florence-2 is generative and does not emit per-box confidence scores;
each returned box is assigned score=1.0 so the downstream consumer (SAM,
debug overlay, JSON response) sees the same shape as before. The
GroundingDINO box/text thresholds in the request are accepted for API
parity but are no-ops here.
"""

from __future__ import annotations

from typing import Any

import torch
from transformers import AutoModelForCausalLM, AutoProcessor


_TASK = "<CAPTION_TO_PHRASE_GROUNDING>"


class FlorenceRunner:
    def __init__(
        self,
        model_id: str = "microsoft/Florence-2-large",
        device: str | None = None,
    ) -> None:
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        self.model_id = model_id

        self._dtype = torch.float16 if device == "cuda" else torch.float32

        self.processor = AutoProcessor.from_pretrained(
            model_id, trust_remote_code=True
        )
        self.model = (
            AutoModelForCausalLM.from_pretrained(
                model_id,
                trust_remote_code=True,
                torch_dtype=self._dtype,
            )
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
        del box_threshold, text_threshold  # unused; kept for API parity

        w, h = pil_image.size
        task_prompt = _TASK + prompt

        inputs = self.processor(
            text=task_prompt, images=pil_image, return_tensors="pt"
        ).to(self.device)
        inputs["pixel_values"] = inputs["pixel_values"].to(self._dtype)

        generated = self.model.generate(
            input_ids=inputs["input_ids"],
            pixel_values=inputs["pixel_values"],
            max_new_tokens=1024,
            num_beams=3,
            do_sample=False,
        )
        text = self.processor.batch_decode(
            generated, skip_special_tokens=False
        )[0]
        parsed = self.processor.post_process_generation(
            text, task=_TASK, image_size=(w, h)
        )

        result = parsed.get(_TASK, {}) or {}
        boxes = result.get("bboxes", []) or []
        labels = result.get("labels", []) or []

        return {
            "boxes":  [[float(v) for v in b] for b in boxes],
            "scores": [1.0] * len(boxes),
            "labels": [str(l) for l in labels],
        }
