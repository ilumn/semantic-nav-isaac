from __future__ import annotations

import math
from collections import defaultdict

from tb3_semantic_map_msgs.msg import SemanticPlace
from .ros_message_utils import coerce_time


def _cluster_entities(records: list, cluster_distance_m: float) -> list[list]:
    clusters: list[list] = []
    visited: set[str] = set()
    for record in records:
        if record.entity_id in visited:
            continue
        cluster = [record]
        visited.add(record.entity_id)
        changed = True
        while changed:
            changed = False
            for candidate in records:
                if candidate.entity_id in visited:
                    continue
                if any(
                    math.dist((candidate.x, candidate.y), (member.x, member.y)) <= cluster_distance_m
                    for member in cluster
                ):
                    cluster.append(candidate)
                    visited.add(candidate.entity_id)
                    changed = True
        clusters.append(cluster)
    return clusters


def build_places(records: list, stamp, frame_id: str, cluster_distance_m: float = 1.5) -> tuple[list[SemanticPlace], dict[str, str]]:
    if not records:
        return [], {}

    places: list[SemanticPlace] = []
    entity_to_place: dict[str, str] = {}
    for index, cluster in enumerate(_cluster_entities(records, cluster_distance_m), start=1):
        xs = [item.x for item in cluster]
        ys = [item.y for item in cluster]
        zs = [item.z for item in cluster]
        labels = [item.semantic_name for item in cluster]
        label_counts = defaultdict(int)
        for label in labels:
            label_counts[label] += 1
        place_type = max(label_counts, key=label_counts.get)
        place_id = f"{place_type}_place_{index:02d}"

        place = SemanticPlace()
        place.header.stamp = coerce_time(stamp)
        place.header.frame_id = frame_id
        place.place_id = place_id
        place.place_type = place_type
        place.member_entity_ids = [item.entity_id for item in cluster]
        place.anchor_pose.position.x = sum(xs) / len(xs)
        place.anchor_pose.position.y = sum(ys) / len(ys)
        place.anchor_pose.position.z = sum(zs) / len(zs)
        place.anchor_pose.orientation.w = 1.0
        place.extent.x = max(xs) - min(xs) if len(xs) > 1 else 0.5
        place.extent.y = max(ys) - min(ys) if len(ys) > 1 else 0.5
        place.extent.z = max(0.5, max(zs) - min(zs) if len(zs) > 1 else 0.5)
        place.confidence = min(0.95, 0.45 + 0.1 * len(cluster))
        places.append(place)

        for item in cluster:
            entity_to_place[item.entity_id] = place_id

    return places, entity_to_place
