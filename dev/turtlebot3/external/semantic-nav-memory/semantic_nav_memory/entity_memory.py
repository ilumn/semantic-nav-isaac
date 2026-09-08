from __future__ import annotations

from collections import defaultdict

import numpy as np

from .config import PipelineConfig
from .models import Entity, Observation, Tracklet
from .ontology import Ontology


def _average_covariance(covariances: list[list[list[float]]]) -> list[list[float]]:
    tensors = np.asarray(covariances, dtype=float)
    return tensors.mean(axis=0).tolist()


def fuse_entities(
    observations: list[Observation],
    tracklets: list[Tracklet],
    ontology: Ontology,
    scene_diagonal: float,
    config: PipelineConfig,
) -> list[Entity]:
    tracklet_by_detection = {
        detection_id: tracklet.tracklet_id
        for tracklet in tracklets
        for detection_id in tracklet.detection_ids
    }
    grouped: dict[str, list[dict]] = defaultdict(list)
    merge_threshold = max(scene_diagonal * config.entity_merge_ratio, scene_diagonal * 0.01)

    for observation in observations:
        label = observation.detector_label
        position = np.asarray(observation.world_pose_estimate, dtype=float)
        best_group = None
        best_distance = float("inf")
        for candidate in grouped[label]:
            distance = float(np.linalg.norm(position - candidate["center"]))
            same_track = tracklet_by_detection.get(observation.detection_id) in candidate["tracklets"]
            if same_track and distance < best_distance:
                best_group = candidate
                best_distance = distance
            elif distance <= merge_threshold and distance < best_distance:
                best_group = candidate
                best_distance = distance

        if best_group is None:
            best_group = {
                "observations": [],
                "tracklets": set(),
                "center": position,
            }
            grouped[label].append(best_group)

        best_group["observations"].append(observation)
        tracklet_id = tracklet_by_detection.get(observation.detection_id)
        if tracklet_id:
            best_group["tracklets"].add(tracklet_id)
        stacked = np.asarray([obs.world_pose_estimate for obs in best_group["observations"]], dtype=float)
        best_group["center"] = stacked.mean(axis=0)

    entities: list[Entity] = []
    for label, groups in grouped.items():
        for index, group in enumerate(groups, start=1):
            obs = group["observations"]
            centers = np.asarray([item.world_pose_estimate for item in obs], dtype=float)
            extents = np.asarray([item.extent_3d for item in obs], dtype=float)
            confidence = float(sum(item.confidence for item in obs) / len(obs))
            support_point3d_ids = sorted({point_id for item in obs for point_id in item.point3d_ids})
            class_distribution: dict[str, float] = defaultdict(float)
            for item in obs:
                for class_name, value in item.class_distribution.items():
                    class_distribution[class_name] += value

            entities.append(
                Entity(
                    entity_id=f"{label.replace(' ', '_')}_{index:02d}",
                    canonical_label=label,
                    class_distribution=dict(class_distribution),
                    aliases=ontology.aliases(label),
                    ontology_parents=ontology.parents(label),
                    world_pose_estimate=centers.mean(axis=0).tolist(),
                    pose_covariance=_average_covariance([item.pose_covariance for item in obs]),
                    extent_3d=np.maximum(extents.mean(axis=0), scene_diagonal * 0.005).tolist(),
                    observed_in_keyframes=sorted({item.keyframe_id for item in obs}),
                    observation_ids=[item.observation_id for item in obs],
                    support_point3d_ids=support_point3d_ids,
                    support_point_count=len(support_point3d_ids),
                    observation_count=len(obs),
                    state="confirmed" if len(obs) >= 2 else "candidate",
                    mobility=ontology.mobility(label),
                    confidence=confidence,
                )
            )
    return entities
