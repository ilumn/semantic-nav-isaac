from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from tb3_localizer.localizer_core import LocalizerCore


@dataclass(slots=True)
class GroundedObservation:
    detector_label: str
    semantic_name: str
    confidence: float
    map_x: float
    map_y: float
    map_z: float
    robot_x: float
    robot_y: float
    bearing_rad: float
    range_m: float
    covariance_xy: tuple[float, float, float]


def transform_camera_to_base(
    camera_x: float,
    camera_y: float,
    camera_tx: float,
    camera_ty: float,
    camera_yaw: float,
) -> tuple[float, float]:
    base_x = camera_tx + math.cos(camera_yaw) * camera_x - math.sin(camera_yaw) * camera_y
    base_y = camera_ty + math.sin(camera_yaw) * camera_x + math.cos(camera_yaw) * camera_y
    return base_x, base_y


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_base_to_map(base_x: float, base_y: float, tx: float, ty: float, yaw: float) -> tuple[float, float]:
    map_x = tx + math.cos(yaw) * base_x - math.sin(yaw) * base_y
    map_y = ty + math.sin(yaw) * base_x + math.cos(yaw) * base_y
    return map_x, map_y


class ObservationGrounder:
    """Ground 2D detections into map-frame observations via scan + TF."""

    def __init__(
        self,
        camera_hfov_deg: float,
        scan_window_half: int,
        min_valid_range: float,
        max_valid_range: float,
        camera_base_tx: float = 0.0,
        camera_base_ty: float = 0.0,
        camera_base_yaw_deg: float = 0.0,
        bearing_sigma_deg: float = 2.0,
        range_sigma_ratio: float = 0.08,
        min_range_sigma_m: float = 0.05,
    ) -> None:
        self.camera_hfov_rad = math.radians(camera_hfov_deg)
        self._localizer = LocalizerCore(
            camera_hfov_rad=self.camera_hfov_rad,
            scan_window_half=scan_window_half,
            min_valid_range=min_valid_range,
            max_valid_range=max_valid_range,
        )
        self._camera_base_tx = float(camera_base_tx)
        self._camera_base_ty = float(camera_base_ty)
        self._camera_base_yaw = math.radians(camera_base_yaw_deg)
        self._bearing_sigma_rad = math.radians(bearing_sigma_deg)
        self._range_sigma_ratio = float(range_sigma_ratio)
        self._min_range_sigma_m = float(min_range_sigma_m)

    def _estimate_covariance_xy(
        self,
        range_m: float,
        bearing_rad: float,
        image_width: int,
        bbox_width_px: float,
    ) -> tuple[float, float, float]:
        width_ratio = 0.0
        if image_width > 0:
            width_ratio = max(0.0, min(1.0, bbox_width_px / float(image_width)))
        bbox_bearing_sigma = max(
            math.radians(0.5),
            (self.camera_hfov_rad * max(width_ratio, 0.02)) / 2.0,
        )
        bearing_sigma = max(self._bearing_sigma_rad, bbox_bearing_sigma)
        range_sigma = max(self._min_range_sigma_m, abs(range_m) * self._range_sigma_ratio)

        cos_b = math.cos(bearing_rad)
        sin_b = math.sin(bearing_rad)
        dxdtheta = -range_m * sin_b
        dydtheta = range_m * cos_b
        dxdr = cos_b
        dydr = sin_b
        var_r = range_sigma * range_sigma
        var_theta = bearing_sigma * bearing_sigma
        cov_xx = dxdr * dxdr * var_r + dxdtheta * dxdtheta * var_theta
        cov_xy = dxdr * dydr * var_r + dxdtheta * dydtheta * var_theta
        cov_yy = dydr * dydr * var_r + dydtheta * dydtheta * var_theta
        return cov_xx, cov_xy, cov_yy

    def ground_detection(
        self,
        detection: dict,
        image_width: int,
        scan_ranges: list[float],
        scan_angle_min: float,
        scan_angle_max: float,
        scan_angle_increment: float,
        robot_tx: float,
        robot_ty: float,
        robot_yaw: float,
        semantic_name: str,
    ) -> Optional[GroundedObservation]:
        x1, _, x2, _ = detection["bbox_xyxy"]
        bbox_center_x = (float(x1) + float(x2)) / 2.0
        bbox_width_px = max(1.0, float(x2) - float(x1))
        localized = self._localizer.localize(
            label=detection["label"],
            confidence=float(detection["conf"]),
            bbox_center_x=bbox_center_x,
            image_width=image_width,
            scan_ranges=scan_ranges,
            scan_angle_min=scan_angle_min,
            scan_angle_max=scan_angle_max,
            scan_angle_increment=scan_angle_increment,
        )
        if localized is None:
            return None

        base_x, base_y = transform_camera_to_base(
            localized.x,
            localized.y,
            self._camera_base_tx,
            self._camera_base_ty,
            self._camera_base_yaw,
        )
        map_x, map_y = transform_base_to_map(
            base_x,
            base_y,
            robot_tx,
            robot_ty,
            robot_yaw,
        )
        covariance_xy = self._estimate_covariance_xy(
            range_m=localized.range_m,
            bearing_rad=localized.bearing_rad + self._camera_base_yaw,
            image_width=image_width,
            bbox_width_px=bbox_width_px,
        )
        confidence_scale = 1.0 / (1.0 + math.sqrt(max(covariance_xy[0], 0.0) + max(covariance_xy[2], 0.0)))
        grounded_confidence = max(0.0, min(1.0, localized.confidence * confidence_scale))
        return GroundedObservation(
            detector_label=localized.label,
            semantic_name=semantic_name,
            confidence=grounded_confidence,
            map_x=map_x,
            map_y=map_y,
            map_z=0.0,
            robot_x=base_x,
            robot_y=base_y,
            bearing_rad=localized.bearing_rad + self._camera_base_yaw,
            range_m=localized.range_m,
            covariance_xy=covariance_xy,
        )
