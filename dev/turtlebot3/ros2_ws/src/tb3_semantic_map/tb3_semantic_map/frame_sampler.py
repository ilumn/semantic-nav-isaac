from __future__ import annotations


class FrameSampler:
    """Simple time-based sampler for image callbacks."""

    def __init__(self, interval_sec: float) -> None:
        self.interval_sec = max(0.0, float(interval_sec))
        self._last_sample_sec = -1.0

    def should_sample(self, stamp_sec: float) -> bool:
        if self._last_sample_sec < 0.0:
            self._last_sample_sec = stamp_sec
            return True
        if self.interval_sec <= 0.0 or stamp_sec - self._last_sample_sec >= self.interval_sec:
            self._last_sample_sec = stamp_sec
            return True
        return False
