from __future__ import annotations

import hashlib
from pathlib import Path

from tb3_detector.detector_core import DetectorCore


class YoloeDetector:
    """Ultralytics YOLOE open-vocabulary detector, text-prompted with the target labels.

    Roughly 18 fps (11l) to 25 fps (11s) on a Jetson Orin NX, versus about one frame every few seconds for
    LocateAnything-3B. The prompted model is cached next to the weights per class list,
    so restarts skip the MobileCLIP text encoder.

    Background classes are prompted but never reported: with one label per box, "wall" or
    "refrigerator" winning a box is what stops it being called a door.
    """

    def __init__(
        self,
        weights: str,
        class_filter: list[str] | None,
        device: str,
        conf: float,
        imgsz: int,
        background_classes: list[str] | None = None,
    ) -> None:
        if not class_filter:
            raise ValueError("YOLOE requires at least one class in class_filter")
        self._background = set(background_classes or [])
        self._weights = Path(weights)
        self._classes = list(class_filter) + [c for c in (background_classes or []) if c not in class_filter]
        self._device = device
        self._conf = float(conf)
        self._imgsz = int(imgsz)
        self._model = None

    def load(self) -> None:
        from ultralytics import YOLOE

        key = hashlib.sha1("\n".join(self._classes).encode()).hexdigest()[:10]
        cached = self._weights.with_name(f"{self._weights.stem}-prompted-{key}.pt")
        if cached.exists():
            self._model = YOLOE(str(cached))
        else:
            self._model = YOLOE(str(self._weights))
            self._model.set_classes(self._classes)
            self._model.save(str(cached))

    def infer(self, bgr_image) -> list[dict]:
        if self._model is None:
            raise RuntimeError("YoloeDetector.load() has not been called yet.")
        result = self._model.predict(
            bgr_image,
            imgsz=self._imgsz,
            conf=self._conf,
            device=self._device,
            half=self._device.startswith("cuda"),
            agnostic_nms=True,  # one label per object, not door + cabinet + can on the same box
            verbose=False,
        )[0]
        boxes = result.boxes
        return [
            {
                "label": result.names[int(cls)],
                "conf": float(score),
                "bbox_xyxy": [float(v) for v in xyxy],
                "track_id": None,
            }
            for xyxy, cls, score in zip(boxes.xyxy.tolist(), boxes.cls.tolist(), boxes.conf.tolist())
            if result.names[int(cls)] not in self._background
        ]

    @property
    def is_loaded(self) -> bool:
        return self._model is not None


class DetectorBridge:
    """Thin wrapper over the configured detector backend: Locate Anything or YOLOE."""

    def __init__(
        self,
        model_id: str,
        model_revision: str,
        class_filter: list[str] | None,
        device: str,
        generation_mode: str,
        max_new_tokens: int,
        local_files_only: bool,
        backend: str = "locate_anything",
        yoloe_weights: str = "yoloe-11s-seg.pt",
        yoloe_conf: float = 0.25,
        yoloe_imgsz: int = 640,
        yoloe_background_classes: list[str] | None = None,
    ) -> None:
        if backend == "yoloe":
            self._core = YoloeDetector(
                weights=yoloe_weights,
                class_filter=class_filter,
                device=device,
                conf=yoloe_conf,
                imgsz=yoloe_imgsz,
                background_classes=yoloe_background_classes,
            )
        elif backend == "locate_anything":
            self._core = DetectorCore(
                model_id=model_id,
                model_revision=model_revision,
                class_filter=class_filter,
                device=device,
                generation_mode=generation_mode,
                max_new_tokens=max_new_tokens,
                local_files_only=local_files_only,
            )
        else:
            raise ValueError(f"Unknown detector_backend {backend!r}; use 'locate_anything' or 'yoloe'")

    def load(self) -> None:
        self._core.load()

    def infer(self, bgr_image) -> list[dict]:
        return self._core.infer(bgr_image)

    @property
    def is_loaded(self) -> bool:
        return self._core.is_loaded
