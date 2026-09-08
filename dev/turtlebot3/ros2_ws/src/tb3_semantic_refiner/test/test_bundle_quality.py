from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tb3_semantic_refiner.bundle_core import BufferedFrame
from tb3_semantic_refiner.bundle_quality import assess_bundle_readiness


def _frame(stamp_sec: float, x: float, y: float, yaw: float) -> BufferedFrame:
    return BufferedFrame(
        stamp_sec=stamp_sec,
        width=640,
        height=480,
        frame_id="camera",
        robot_x=x,
        robot_y=y,
        robot_yaw=yaw,
    )


def test_assess_bundle_readiness_blocks_static_pose_bundle():
    frames = [_frame(1.0, 0.0, 0.0, 0.0), _frame(2.0, 0.0, 0.0, 0.0), _frame(3.0, 0.0, 0.0, 0.0)]

    readiness = assess_bundle_readiness(
        frames,
        min_pose_frames=3,
        min_translation_span_m=0.35,
        min_path_length_m=0.75,
    )

    assert not readiness.ready
    assert readiness.reason == "waiting_for_motion_baseline"


def test_assess_bundle_readiness_accepts_bundle_with_baseline():
    frames = [_frame(1.0, 0.0, 0.0, 0.0), _frame(2.0, 0.2, 0.0, 0.05), _frame(3.0, 0.45, 0.1, 0.08)]

    readiness = assess_bundle_readiness(
        frames,
        min_pose_frames=3,
        min_translation_span_m=0.35,
        min_path_length_m=0.75,
    )

    assert readiness.ready
    assert readiness.translation_span_m >= 0.35
