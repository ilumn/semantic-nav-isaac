from __future__ import annotations

from collections import defaultdict

import numpy as np

from .geometry_colmap import GeometryResult
from .models import Detection, Observation


def _points2d_for_image(image):
    points_attr = image.points2D
    return points_attr() if callable(points_attr) else points_attr


def _inside(bbox, xy: np.ndarray) -> bool:
    return bbox.x1 <= float(xy[0]) <= bbox.x2 and bbox.y1 <= float(xy[1]) <= bbox.y2


def _covariance(points: np.ndarray, fallback_scale: float) -> list[list[float]]:
    if len(points) >= 2:
        cov = np.cov(points, rowvar=False)
    else:
        scale = max(fallback_scale * 0.01, 1e-4)
        cov = np.diag([scale, scale, scale])
    return np.asarray(cov, dtype=float).tolist()


def ground_detections(
    detections: list[Detection],
    geometry: GeometryResult,
) -> list[Observation]:
    detections_by_keyframe = defaultdict(list)
    for detection in detections:
        detections_by_keyframe[detection.keyframe_id].append(detection)

    image_by_keyframe = {
        camera.keyframe_id: geometry.reconstruction.find_image_with_name(camera.image_name)
        for camera in geometry.artifact.cameras
    }

    observations: list[Observation] = []
    for keyframe_id, keyframe_detections in detections_by_keyframe.items():
        image = image_by_keyframe.get(keyframe_id)
        if image is None:
            continue
        points2d = _points2d_for_image(image)
        visible_points: list[tuple[int, np.ndarray, np.ndarray]] = []
        for point2d in points2d:
            if not point2d.has_point3D():
                continue
            point_id = int(point2d.point3D_id)
            xyz = geometry.point_lookup.get(point_id)
            if xyz is None:
                continue
            visible_points.append((point_id, np.asarray(point2d.xy, dtype=float), xyz))

        for obs_index, detection in enumerate(keyframe_detections, start=1):
            supporting_points = [
                (point_id, xyz)
                for point_id, xy, xyz in visible_points
                if _inside(detection.bbox, xy)
            ]
            if not supporting_points:
                continue

            point_ids = [point_id for point_id, _ in supporting_points]
            points = np.asarray([xyz for _, xyz in supporting_points], dtype=float)
            centroid = points.mean(axis=0)
            extent = np.ptp(points, axis=0) if len(points) > 1 else np.full(3, geometry.scene_diagonal * 0.01)
            observations.append(
                Observation(
                    observation_id=f"obs_{keyframe_id}_{obs_index:03d}",
                    keyframe_id=keyframe_id,
                    detection_id=detection.detection_id,
                    detector_label=detection.detector_label,
                    class_distribution=detection.class_distribution,
                    bbox=detection.bbox,
                    confidence=detection.confidence,
                    world_pose_estimate=centroid.astype(float).tolist(),
                    pose_covariance=_covariance(points, geometry.scene_diagonal),
                    extent_3d=np.maximum(extent, geometry.scene_diagonal * 0.005).astype(float).tolist(),
                    point3d_ids=point_ids,
                    support_point_count=len(point_ids),
                )
            )
    return observations
