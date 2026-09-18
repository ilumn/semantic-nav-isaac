#!/usr/bin/env python3
"""Stage-1 perception backed by NVIDIA LocateAnything-3B."""

from __future__ import annotations

import logging
import re
import sys
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def _extend_sys_path_from_repo_venv() -> None:
    """Allow ROS nodes running under system Python to reuse repo-local .venv deps."""
    for parent in Path(__file__).resolve().parents:
        venv = parent / ".venv"
        if not venv.is_dir():
            continue
        lib_dir = venv / "lib"
        if not lib_dir.is_dir():
            continue
        for candidate in sorted(lib_dir.glob("python*/site-packages")):
            candidate_str = str(candidate)
            if candidate_str not in sys.path:
                sys.path.insert(0, candidate_str)
        break


_extend_sys_path_from_repo_venv()

try:
    import torch
    from PIL import Image
    from transformers import AutoModel, AutoProcessor, AutoTokenizer

    _LOCATE_ANYTHING_AVAILABLE = True
except ImportError:
    torch = None
    Image = None
    AutoModel = None
    AutoProcessor = None
    AutoTokenizer = None
    _LOCATE_ANYTHING_AVAILABLE = False


DETECTION_KEYS = ("label", "conf", "bbox_xyxy", "track_id")
DEFAULT_MODEL_ID = "nvidia/LocateAnything-3B"
DEFAULT_MODEL_REVISION = "c32291ca5e996f5a7a485845b4f57a233936bba0"

_OUTPUT_TOKEN_RE = re.compile(
    r"<ref>(?P<label>.*?)</ref>|"
    r"<box><(?P<x1>\d+)><(?P<y1>\d+)><(?P<x2>\d+)><(?P<y2>\d+)></box>"
)


def parse_locate_anything_boxes(
    answer: str,
    image_width: int,
    image_height: int,
    requested_labels: list[str],
) -> list[dict]:
    """Convert Locate Anything's normalized tagged boxes to detector records."""
    label_lookup = {label.casefold(): label for label in requested_labels}
    current_label = requested_labels[0] if len(requested_labels) == 1 else ""
    detections: list[dict] = []

    for match in _OUTPUT_TOKEN_RE.finditer(answer):
        raw_label = match.group("label")
        if raw_label is not None:
            raw_label = raw_label.strip()
            current_label = label_lookup.get(raw_label.casefold(), "")
            if not current_label:
                logger.warning(
                    "Ignoring Locate Anything output for unrequested label %r", raw_label
                )
            continue

        if not current_label:
            logger.warning("Ignoring Locate Anything box without a label: %s", answer)
            continue

        values = [int(match.group(name)) for name in ("x1", "y1", "x2", "y2")]
        x1, y1, x2, y2 = [min(1000, max(0, value)) for value in values]
        if x2 <= x1 or y2 <= y1:
            logger.warning("Ignoring invalid Locate Anything box %s", values)
            continue

        detections.append(
            {
                "label": current_label,
                # Locate Anything emits boxes without calibrated confidence scores.
                # 1.0 means the model returned this box; it is not a probability.
                "conf": 1.0,
                "bbox_xyxy": [
                    x1 / 1000.0 * image_width,
                    y1 / 1000.0 * image_height,
                    x2 / 1000.0 * image_width,
                    y2 / 1000.0 * image_height,
                ],
                "track_id": None,
            }
        )

    return detections


class DetectorCore:
    """Load LocateAnything-3B once and run open-vocabulary box detection."""

    def __init__(
        self,
        model_id: str = DEFAULT_MODEL_ID,
        model_revision: str = DEFAULT_MODEL_REVISION,
        class_filter: list[str] | None = None,
        device: str = "cuda:0",
        generation_mode: str = "hybrid",
        max_new_tokens: int = 2048,
        local_files_only: bool = True,
    ) -> None:
        self.model_id = model_id
        self.model_revision = model_revision
        self.class_filter = (
            list(class_filter)
            if class_filter is not None
            else ["bench", "person", "stop sign"]
        )
        self.device = device
        self.generation_mode = generation_mode
        self.max_new_tokens = int(max_new_tokens)
        self.local_files_only = bool(local_files_only)

        if not self.class_filter:
            raise ValueError("Locate Anything requires at least one class in class_filter")
        if self.generation_mode not in {"fast", "slow", "hybrid"}:
            raise ValueError("generation_mode must be one of: fast, slow, hybrid")
        if self.max_new_tokens <= 0:
            raise ValueError("max_new_tokens must be positive")

        self._model: Any = None
        self._processor: Any = None
        self._tokenizer: Any = None

    def load(self) -> None:
        if not _LOCATE_ANYTHING_AVAILABLE:
            raise RuntimeError(
                "Locate Anything dependencies are unavailable. Run "
                "dev/isaac_sim/bootstrap_runtime.sh --allow-download."
            )

        model_source = str(Path(self.model_id).resolve()) if Path(self.model_id).exists() else self.model_id
        load_kwargs = {
            "revision": self.model_revision,
            "trust_remote_code": True,
            "local_files_only": self.local_files_only,
        }
        logger.info(
            "Loading Locate Anything model %s at %s on %s",
            model_source,
            self.model_revision,
            self.device,
        )
        self._tokenizer = AutoTokenizer.from_pretrained(model_source, **load_kwargs)
        self._processor = AutoProcessor.from_pretrained(
            model_source, use_fast=False, **load_kwargs
        )
        self._dtype = torch.bfloat16 if self.device.startswith("cuda") else torch.float32
        self._model = AutoModel.from_pretrained(
            model_source,
            dtype=self._dtype,
            **load_kwargs,
        ).to(self.device).eval()
        logger.info("Locate Anything loaded for categories: %s", self.class_filter)

    @torch.no_grad() if torch is not None else (lambda function: function)
    def infer(self, bgr_image) -> list[dict]:
        if self._model is None or self._processor is None or self._tokenizer is None:
            raise RuntimeError("DetectorCore.load() has not been called yet.")

        rgb_image = Image.fromarray(bgr_image[:, :, ::-1]).convert("RGB")
        categories = "</c>".join(self.class_filter)
        question = (
            "Locate all the instances that matches the following description: "
            f"{categories}."
        )
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": rgb_image},
                    {"type": "text", "text": question},
                ],
            }
        ]
        text = self._processor.py_apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )
        images, videos = self._processor.process_vision_info(messages)
        inputs = self._processor(
            text=[text], images=images, videos=videos, return_tensors="pt"
        ).to(self.device)
        pixel_values = inputs["pixel_values"].to(self._dtype)

        response = self._model.generate(
            pixel_values=pixel_values,
            input_ids=inputs["input_ids"],
            attention_mask=inputs["attention_mask"],
            image_grid_hws=inputs.get("image_grid_hws"),
            tokenizer=self._tokenizer,
            max_new_tokens=self.max_new_tokens,
            use_cache=True,
            generation_mode=self.generation_mode,
            temperature=0.7,
            do_sample=True,
            top_p=0.9,
            repetition_penalty=1.1,
            verbose=False,
        )
        answer = response[0] if isinstance(response, tuple) else response
        if not isinstance(answer, str):
            answer = str(answer)
        return parse_locate_anything_boxes(
            answer, rgb_image.width, rgb_image.height, self.class_filter
        )

    @property
    def is_loaded(self) -> bool:
        return self._model is not None
