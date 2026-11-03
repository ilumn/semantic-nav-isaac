from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SRC_ROOT / "tb3_localizer"))

from tb3_semantic_map.grounding import (
    ObservationGrounder,
    transform_base_to_map,
    transform_camera_to_base,
)


def test_transform_base_to_map_applies_translation_and_rotation():
    map_x, map_y = transform_base_to_map(1.0, 0.0, 2.0, 3.0, math.pi / 2.0)
    assert abs(map_x - 2.0) < 1e-6
    assert abs(map_y - 4.0) < 1e-6


def test_ground_detection_returns_map_frame_point():
    grounder = ObservationGrounder(
        camera_hfov_deg=62.2,
        scan_window_half=1,
        min_valid_range=0.12,
        max_valid_range=8.0,
    )
    detection = {"label": "person", "conf": 0.9, "bbox_xyxy": [300.0, 100.0, 340.0, 200.0]}
    scan_ranges = [2.0] * 360
    result = grounder.ground_detection(
        detection=detection,
        image_width=640,
        scan_ranges=scan_ranges,
        scan_angle_min=0.0,
        scan_angle_max=2.0 * math.pi,
        scan_angle_increment=(2.0 * math.pi) / 360.0,
        robot_tx=1.0,
        robot_ty=2.0,
        robot_yaw=0.0,
        semantic_name="person",
    )

    assert result is not None
    assert result.semantic_name == "person"
    assert result.detector_label == "person"
    assert 0.0 < result.confidence <= 0.9
    assert result.map_x > 2.5
    assert abs(result.map_y - 2.0) < 0.5


def test_transform_camera_to_base_applies_camera_offset_and_yaw():
    base_x, base_y = transform_camera_to_base(
        camera_x=1.0,
        camera_y=0.0,
        camera_tx=0.2,
        camera_ty=0.1,
        camera_yaw=math.pi / 2.0,
    )
    assert abs(base_x - 0.2) < 1e-6
    assert abs(base_y - 1.1) < 1e-6


def test_ground_detection_reports_covariance_and_applies_camera_extrinsic():
    grounder = ObservationGrounder(
        camera_hfov_deg=62.2,
        scan_window_half=1,
        min_valid_range=0.12,
        max_valid_range=8.0,
        camera_base_tx=0.2,
        camera_base_ty=0.0,
        camera_base_yaw_deg=0.0,
    )
    detection = {"label": "person", "conf": 0.9, "bbox_xyxy": [300.0, 100.0, 340.0, 200.0]}
    scan_ranges = [2.0] * 360
    result = grounder.ground_detection(
        detection=detection,
        image_width=640,
        scan_ranges=scan_ranges,
        scan_angle_min=0.0,
        scan_angle_max=2.0 * math.pi,
        scan_angle_increment=(2.0 * math.pi) / 360.0,
        robot_tx=1.0,
        robot_ty=2.0,
        robot_yaw=0.0,
        semantic_name="person",
    )

    assert result is not None
    assert result.robot_x > 2.0
    assert result.covariance_xy[0] > 0.0
    assert result.covariance_xy[2] > 0.0
    assert 0.0 < result.confidence <= 0.9


def _real_robot_grounder():
    return ObservationGrounder(
        camera_hfov_deg=60.0,
        scan_window_half=5,
        min_valid_range=0.12,
        max_valid_range=8.0,
        camera_base_tx=0.073,
        camera_base_ty=0.0,
        lidar_base_tx=-0.064,
    )


def _ground(grounder, label, x1, x2, ranges):
    return grounder.ground_detection(
        detection={"label": label, "conf": 0.9, "bbox_xyxy": [x1, 0.0, x2, 100.0]},
        image_width=640,
        scan_ranges=ranges,
        scan_angle_min=0.0,
        scan_angle_max=2.0 * math.pi,
        scan_angle_increment=2.0 * math.pi / 360,
        robot_tx=0.0,
        robot_ty=0.0,
        robot_yaw=0.0,
        semantic_name=label,
    )


def test_box_touching_image_side_is_not_grounded():
    assert _ground(_real_robot_grounder(), "chair", 0.0, 200.0, [2.0] * 360) is None


def test_object_too_wide_for_its_class_is_rejected():
    assert _ground(_real_robot_grounder(), "soda can", 200.0, 440.0, [2.0] * 360) is None


def test_range_is_measured_from_the_lidar_position():
    obs = _ground(_real_robot_grounder(), "chair", 280.0, 360.0, [2.0] * 360)
    assert obs is not None
    # A wall 2 m from the lidar (at x=-0.064) is at about x=1.936 in base_link.
    assert abs(obs.robot_x - 1.936) < 0.05


def test_nearest_surface_wins_over_the_wall_behind():
    ranges = [3.0] * 360
    for i in (-2, -1, 0, 1, 2):
        ranges[i % 360] = 1.0
    obs = _ground(_real_robot_grounder(), "chair", 240.0, 400.0, ranges)
    assert obs is not None
    assert obs.robot_x < 1.2
