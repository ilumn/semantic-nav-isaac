from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Optional


@dataclass(slots=True)
class BufferedFrame:
    stamp_sec: float
    width: int
    height: int
    frame_id: str
    robot_x: Optional[float] = None
    robot_y: Optional[float] = None
    robot_yaw: Optional[float] = None
    fx: Optional[float] = None
    fy: Optional[float] = None
    cx: Optional[float] = None
    cy: Optional[float] = None


class FrameBundleBuffer:
    """Rolling buffer for sampled frame metadata used by async refinement."""

    def __init__(self, max_frames: int) -> None:
        self._frames: deque[BufferedFrame] = deque(maxlen=max(1, int(max_frames)))

    def add(self, frame: BufferedFrame) -> None:
        self._frames.append(frame)

    def snapshot(self) -> list[BufferedFrame]:
        return list(self._frames)

    def __len__(self) -> int:
        return len(self._frames)
