from __future__ import annotations

import math
from dataclasses import dataclass

from .bundle_core import BufferedFrame


def _shortest_angle_delta_rad(a: float, b: float) -> float:
    delta = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return abs(delta)


@dataclass(slots=True)
class BundleReadiness:
    total_frames: int
    pose_frames: int
    translation_span_m: float
    path_length_m: float
    yaw_span_rad: float
    ready: bool
    reason: str


def assess_bundle_readiness(
    frames: list[BufferedFrame],
    min_pose_frames: int,
    min_translation_span_m: float,
    min_path_length_m: float,
) -> BundleReadiness:
    pose_frames = [
        frame
        for frame in frames
        if frame.robot_x is not None and frame.robot_y is not None and frame.robot_yaw is not None
    ]
    if len(pose_frames) < max(1, int(min_pose_frames)):
        return BundleReadiness(
            total_frames=len(frames),
            pose_frames=len(pose_frames),
            translation_span_m=0.0,
            path_length_m=0.0,
            yaw_span_rad=0.0,
            ready=False,
            reason="waiting_for_pose_baseline",
        )

    first = pose_frames[0]
    translation_span_m = 0.0
    path_length_m = 0.0
    yaw_span_rad = 0.0
    previous = first
    for frame in pose_frames[1:]:
        translation_span_m = max(
            translation_span_m,
            math.hypot(frame.robot_x - first.robot_x, frame.robot_y - first.robot_y),
        )
        path_length_m += math.hypot(frame.robot_x - previous.robot_x, frame.robot_y - previous.robot_y)
        yaw_span_rad = max(yaw_span_rad, _shortest_angle_delta_rad(first.robot_yaw, frame.robot_yaw))
        previous = frame

    translation_ready = translation_span_m >= max(0.0, float(min_translation_span_m))
    path_ready = path_length_m >= max(0.0, float(min_path_length_m))
    ready = translation_ready or path_ready
    reason = "ready" if ready else "waiting_for_motion_baseline"
    return BundleReadiness(
        total_frames=len(frames),
        pose_frames=len(pose_frames),
        translation_span_m=translation_span_m,
        path_length_m=path_length_m,
        yaw_span_rad=yaw_span_rad,
        ready=ready,
        reason=reason,
    )
