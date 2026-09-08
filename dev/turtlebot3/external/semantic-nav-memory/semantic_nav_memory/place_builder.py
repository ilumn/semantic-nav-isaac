from __future__ import annotations

from collections import defaultdict

import numpy as np

from .config import PipelineConfig
from .models import Entity, Place
from .ontology import Ontology


def build_places(
    entities: list[Entity],
    ontology: Ontology,
    scene_diagonal: float,
    config: PipelineConfig,
) -> list[Place]:
    if not entities:
        return []

    threshold = max(scene_diagonal * config.place_cluster_ratio, scene_diagonal * 0.04)
    positions = {entity.entity_id: np.asarray(entity.world_pose_estimate, dtype=float) for entity in entities}

    neighbors: dict[str, set[str]] = {entity.entity_id: set() for entity in entities}
    for left in entities:
        for right in entities:
            if left.entity_id >= right.entity_id:
                continue
            distance = float(np.linalg.norm(positions[left.entity_id] - positions[right.entity_id]))
            if distance <= threshold:
                neighbors[left.entity_id].add(right.entity_id)
                neighbors[right.entity_id].add(left.entity_id)

    visited: set[str] = set()
    places: list[Place] = []
    entity_by_id = {entity.entity_id: entity for entity in entities}

    for entity in entities:
        if entity.entity_id in visited:
            continue
        cluster = []
        stack = [entity.entity_id]
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            cluster.append(current)
            stack.extend(neighbors[current] - visited)

        cluster_entities = [entity_by_id[entity_id] for entity_id in cluster]
        anchor = np.asarray([member.world_pose_estimate for member in cluster_entities], dtype=float).mean(axis=0)
        member_centers = np.asarray([member.world_pose_estimate for member in cluster_entities], dtype=float)
        member_extents = np.asarray([member.extent_3d for member in cluster_entities], dtype=float)
        min_corner = (member_centers - member_extents * 0.5).min(axis=0)
        max_corner = (member_centers + member_extents * 0.5).max(axis=0)
        place_scores: dict[str, float] = defaultdict(float)
        observed = set()
        support_surfaces = []
        for member in cluster_entities:
            observed.update(member.observed_in_keyframes)
            if "support_surface" in member.ontology_parents or member.canonical_label in {
                "desk",
                "table",
                "cabinet",
                "road",
                "driveway",
                "lawn",
                "roof",
            }:
                support_surfaces.append(member.entity_id)
            for place_prior in ontology.place_priors(member.canonical_label):
                place_scores[place_prior] += member.confidence
        if not place_scores:
            place_scores["room_cluster"] = 0.5

        normalizer = max(place_scores.values())
        distribution = {name: round(value / normalizer, 3) for name, value in place_scores.items()}
        place_id = f"{max(distribution, key=distribution.get)}_place_{len(places) + 1:02d}"
        for member in cluster_entities:
            member.place_id = place_id
        places.append(
            Place(
                place_id=place_id,
                place_type_distribution=distribution,
                anchor_pose=anchor.tolist(),
                extent_3d=(max_corner - min_corner).tolist(),
                member_entities=[member.entity_id for member in cluster_entities],
                support_surfaces=support_surfaces,
                observed_in_keyframes=sorted(observed),
                confidence=min(0.95, 0.45 + 0.08 * len(cluster_entities)),
            )
        )
    return places
