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
