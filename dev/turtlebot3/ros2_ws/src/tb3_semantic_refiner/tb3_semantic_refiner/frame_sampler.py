from __future__ import annotations

import math
from typing import Optional


def _shortest_angle_delta_rad(a: float, b: float) -> float:
    delta = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return abs(delta)


class FrameSampler:
    """Time-based sampler with optional motion gating for image callbacks."""

    def __init__(
        self,
        interval_sec: float,
        min_translation_m: float = 0.0,
        min_yaw_delta_rad: float = 0.0,
    ) -> None:
        self.interval_sec = max(0.0, float(interval_sec))
        self.min_translation_m = max(0.0, float(min_translation_m))
        self.min_yaw_delta_rad = max(0.0, float(min_yaw_delta_rad))
        self._last_sample_sec = -1.0
        self._last_robot_x: Optional[float] = None
        self._last_robot_y: Optional[float] = None
        self._last_robot_yaw: Optional[float] = None

    def should_sample(
        self,
        stamp_sec: float,
        robot_x: Optional[float] = None,
        robot_y: Optional[float] = None,
        robot_yaw: Optional[float] = None,
    ) -> bool:
        if self._last_sample_sec < 0.0:
            self._record_sample(stamp_sec, robot_x, robot_y, robot_yaw)
            return True
        if self.interval_sec > 0.0 and stamp_sec - self._last_sample_sec < self.interval_sec:
            return False
        if not self._meets_motion_gate(robot_x, robot_y, robot_yaw):
            return False
        self._record_sample(stamp_sec, robot_x, robot_y, robot_yaw)
        return True

    def _record_sample(
        self,
        stamp_sec: float,
        robot_x: Optional[float],
        robot_y: Optional[float],
        robot_yaw: Optional[float],
    ) -> None:
        self._last_sample_sec = stamp_sec
        self._last_robot_x = robot_x
        self._last_robot_y = robot_y
        self._last_robot_yaw = robot_yaw

    def _meets_motion_gate(
        self,
        robot_x: Optional[float],
        robot_y: Optional[float],
        robot_yaw: Optional[float],
    ) -> bool:
        if self.min_translation_m <= 0.0 and self.min_yaw_delta_rad <= 0.0:
            return True
        if robot_x is None or robot_y is None or robot_yaw is None:
            return True
        if self._last_robot_x is None or self._last_robot_y is None or self._last_robot_yaw is None:
            return True

        translation = math.hypot(robot_x - self._last_robot_x, robot_y - self._last_robot_y)
        yaw_delta = _shortest_angle_delta_rad(self._last_robot_yaw, robot_yaw)
        if translation >= self.min_translation_m:
            return True
        if yaw_delta >= self.min_yaw_delta_rad:
            return True
        return False
