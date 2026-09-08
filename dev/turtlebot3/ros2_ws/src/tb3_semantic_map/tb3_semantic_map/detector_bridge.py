from __future__ import annotations

import os
from pathlib import Path

from tb3_detector.detector_core import DetectorCore


def resolve_model_path(model_path_raw: str) -> str:
    path = Path(model_path_raw)
    if path.is_absolute():
        return str(path)
    detector_pkg_dir = Path(__file__).resolve().parents[2] / "tb3_detector"
    candidate = detector_pkg_dir / "models" / model_path_raw
    if candidate.exists():
        return str(candidate)
    repo_root = Path(__file__).resolve().parents[6]
    repo_candidate = repo_root / model_path_raw
    if repo_candidate.exists():
        return str(repo_candidate)
    return str(path)


class DetectorBridge:
    """Thin wrapper that reuses the existing YOLOv8 detector core."""

    def __init__(
        self,
        model_path: str,
        conf_threshold: float,
        class_filter: list[str] | None,
        device: str,
        enable_tracking: bool,
    ) -> None:
        self._core = DetectorCore(
            model_path=resolve_model_path(model_path),
            conf_threshold=conf_threshold,
            class_filter=class_filter,
            device=device,
            enable_tracking=enable_tracking,
        )

    def load(self) -> None:
        self._core.load()

    def infer(self, bgr_image) -> list[dict]:
        return self._core.infer(bgr_image)

    @property
    def is_loaded(self) -> bool:
        return self._core.is_loaded
