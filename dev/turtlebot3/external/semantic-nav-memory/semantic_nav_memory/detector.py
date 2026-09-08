from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path

from .models import Detection, Keyframe


class Detector(ABC):
    @abstractmethod
    def detect(self, keyframes: list[Keyframe], output_dir: Path) -> list[Detection]:
        raise NotImplementedError
