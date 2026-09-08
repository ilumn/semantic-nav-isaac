from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tb3_semantic_refiner.frame_sampler import FrameSampler


def test_frame_sampler_rejects_static_frames_after_first_sample():
    sampler = FrameSampler(1.0, min_translation_m=0.1, min_yaw_delta_rad=0.2)

    assert sampler.should_sample(1.0, 0.0, 0.0, 0.0)
    assert not sampler.should_sample(2.1, 0.0, 0.0, 0.0)
    assert sampler.should_sample(3.2, 0.11, 0.0, 0.0)


def test_frame_sampler_accepts_rotation_baseline():
    sampler = FrameSampler(1.0, min_translation_m=0.1, min_yaw_delta_rad=0.2)

    assert sampler.should_sample(1.0, 0.0, 0.0, 0.0)
    assert sampler.should_sample(2.2, 0.0, 0.0, 0.25)
