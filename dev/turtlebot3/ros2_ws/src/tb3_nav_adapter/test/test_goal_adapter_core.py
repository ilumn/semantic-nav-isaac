from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tb3_nav_adapter.goal_adapter_core import compute_approach_pose, compute_approach_pose_global


def test_compute_approach_pose_base_link():
    result = compute_approach_pose(2.0, 0.0, approach_distance=0.5, min_standoff=0.3)

    assert result is not None
    gx, gy, yaw = result
    assert math.isclose(gx, 1.5)
    assert math.isclose(gy, 0.0)
    assert math.isclose(yaw, 0.0)


def test_compute_approach_pose_global_uses_robot_position():
    result = compute_approach_pose_global(
        target_x=4.0,
        target_y=2.0,
        robot_x=1.0,
        robot_y=2.0,
        approach_distance=0.5,
        min_standoff=0.3,
    )

    assert result is not None
    gx, gy, yaw = result
    assert math.isclose(gx, 3.5)
    assert math.isclose(gy, 2.0)
    assert math.isclose(yaw, 0.0)


def test_compute_approach_pose_global_rejects_too_close_target():
    result = compute_approach_pose_global(
        target_x=1.1,
        target_y=1.0,
        robot_x=1.0,
        robot_y=1.0,
        approach_distance=0.5,
        min_standoff=0.3,
    )

    assert result is None
