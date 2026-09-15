from __future__ import annotations

from tb3_detector.detector_core import DetectorCore


class DetectorBridge:
    """Thin wrapper that reuses the Locate Anything detector core."""

    def __init__(
        self,
        model_id: str,
        model_revision: str,
        class_filter: list[str] | None,
        device: str,
        generation_mode: str,
        max_new_tokens: int,
        local_files_only: bool,
    ) -> None:
        self._core = DetectorCore(
            model_id=model_id,
            model_revision=model_revision,
            class_filter=class_filter,
            device=device,
            generation_mode=generation_mode,
            max_new_tokens=max_new_tokens,
            local_files_only=local_files_only,
        )

    def load(self) -> None:
        self._core.load()

    def infer(self, bgr_image) -> list[dict]:
        return self._core.infer(bgr_image)

    @property
    def is_loaded(self) -> bool:
        return self._core.is_loaded
