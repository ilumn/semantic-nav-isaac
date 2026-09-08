from __future__ import annotations

import numpy as np

from .config import PipelineConfig
from .models import Entity, Place, Relation
from .ontology import Ontology


def infer_relations(
    entities: list[Entity],
    places: list[Place],
    ontology: Ontology,
    scene_diagonal: float,
    config: PipelineConfig,
) -> list[Relation]:
    relations: list[Relation] = []
    near_threshold = max(scene_diagonal * config.relation_near_ratio, scene_diagonal * 0.03)
    place_by_id = {place.place_id: place for place in places}
    seen_triplets: set[tuple[str, str, str]] = set()

    def add_relation(subject: str, predicate: str, object_: str, confidence: float, evidence_keyframes: list[str]) -> None:
        triplet = (subject, predicate, object_)
        if triplet in seen_triplets:
            return
        seen_triplets.add(triplet)
        relations.append(
            Relation(
                subject=subject,
                predicate=predicate,
                object=object_,
                confidence=confidence,
                evidence_keyframes=evidence_keyframes,
            )
        )

    for entity in entities:
        if entity.place_id and entity.place_id in place_by_id:
            add_relation(
                entity.entity_id,
                "in_place",
                entity.place_id,
                min(0.98, 0.6 + entity.confidence * 0.3),
                entity.observed_in_keyframes,
            )
            add_relation(
                entity.entity_id,
                "seen_from_place",
                entity.place_id,
                min(0.9, 0.5 + entity.confidence * 0.25),
                entity.observed_in_keyframes,
            )

    for index, left in enumerate(entities):
        left_xyz = np.asarray(left.world_pose_estimate, dtype=float)
        left_extent = np.asarray(left.extent_3d, dtype=float)
        for right in entities[index + 1 :]:
            right_xyz = np.asarray(right.world_pose_estimate, dtype=float)
            right_extent = np.asarray(right.extent_3d, dtype=float)
            delta = right_xyz - left_xyz
            distance = float(np.linalg.norm(delta))
            shared_keyframes = sorted(set(left.observed_in_keyframes) & set(right.observed_in_keyframes))
            evidence = shared_keyframes or sorted(set(left.observed_in_keyframes + right.observed_in_keyframes))[:3]
            horizontal_distance = float(np.linalg.norm(delta[:2]))
            vertical_gap = float(delta[2])

            if distance <= near_threshold:
                add_relation(
                    left.entity_id,
                    "near",
                    right.entity_id,
                    max(0.35, 1.0 - distance / near_threshold),
                    evidence,
                )
                add_relation(
                    right.entity_id,
                    "near",
                    left.entity_id,
                    max(0.35, 1.0 - distance / near_threshold),
                    evidence,
                )

            if abs(delta[0]) > max(abs(delta[1]), abs(delta[2])) and abs(delta[0]) > scene_diagonal * 0.02:
                if delta[0] > 0:
                    add_relation(left.entity_id, "left_of", right.entity_id, 0.45, evidence)
                    add_relation(right.entity_id, "right_of", left.entity_id, 0.45, evidence)
                else:
                    add_relation(right.entity_id, "left_of", left.entity_id, 0.45, evidence)
                    add_relation(left.entity_id, "right_of", right.entity_id, 0.45, evidence)

            if abs(delta[1]) > max(abs(delta[0]), abs(delta[2])) and abs(delta[1]) > scene_diagonal * 0.02:
                if delta[1] > 0:
                    add_relation(left.entity_id, "in_front_of", right.entity_id, 0.42, evidence)
                    add_relation(right.entity_id, "behind", left.entity_id, 0.42, evidence)
                else:
                    add_relation(right.entity_id, "in_front_of", left.entity_id, 0.42, evidence)
                    add_relation(left.entity_id, "behind", right.entity_id, 0.42, evidence)

            support_priors = set(ontology.support_priors(left.canonical_label))
            if right.canonical_label in support_priors and vertical_gap > 0 and horizontal_distance < near_threshold * 0.7:
                add_relation(left.entity_id, "on", right.entity_id, 0.4, evidence)
                add_relation(left.entity_id, "on_top_of", right.entity_id, 0.4, evidence)
                add_relation(right.entity_id, "under", left.entity_id, 0.4, evidence)
                left.support_surface_id = right.entity_id
            elif vertical_gap > 0 and horizontal_distance < max(right_extent[0], right_extent[1], near_threshold * 0.4):
                add_relation(left.entity_id, "on", right.entity_id, 0.32, evidence)
                add_relation(left.entity_id, "on_top_of", right.entity_id, 0.32, evidence)
                add_relation(right.entity_id, "under", left.entity_id, 0.32, evidence)
                left.support_surface_id = right.entity_id

            support_priors = set(ontology.support_priors(right.canonical_label))
            if left.canonical_label in support_priors and vertical_gap < 0 and horizontal_distance < near_threshold * 0.7:
                add_relation(right.entity_id, "on", left.entity_id, 0.4, evidence)
                add_relation(right.entity_id, "on_top_of", left.entity_id, 0.4, evidence)
                add_relation(left.entity_id, "under", right.entity_id, 0.4, evidence)
                right.support_surface_id = left.entity_id
            elif vertical_gap < 0 and horizontal_distance < max(left_extent[0], left_extent[1], near_threshold * 0.4):
                add_relation(right.entity_id, "on", left.entity_id, 0.32, evidence)
                add_relation(right.entity_id, "on_top_of", left.entity_id, 0.32, evidence)
                add_relation(left.entity_id, "under", right.entity_id, 0.32, evidence)
                right.support_surface_id = left.entity_id

            larger, smaller = (left, right) if np.prod(left_extent) >= np.prod(right_extent) else (right, left)
            larger_xyz = np.asarray(larger.world_pose_estimate, dtype=float)
            smaller_xyz = np.asarray(smaller.world_pose_estimate, dtype=float)
            larger_extent = np.asarray(larger.extent_3d, dtype=float)
            displacement = np.abs(smaller_xyz - larger_xyz)
            if np.all(displacement <= np.maximum(larger_extent * 0.6, scene_diagonal * 0.015)):
                if larger.canonical_label in {"cabinet", "backpack"} or "container" in ontology.parents(larger.canonical_label):
                    add_relation(smaller.entity_id, "inside", larger.entity_id, 0.34, evidence)
                elif larger.place_id and smaller.place_id and larger.place_id == smaller.place_id:
                    add_relation(smaller.entity_id, "inside", larger.entity_id, 0.22, evidence)
    return relations
