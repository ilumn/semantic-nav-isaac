from __future__ import annotations

import math

from tb3_semantic_map_msgs.msg import SemanticRelation
from .ros_message_utils import coerce_time


def build_relations(records: list, stamp, frame_id: str, near_distance_m: float = 1.5) -> list[SemanticRelation]:
    relations: list[SemanticRelation] = []
    for left_index, left in enumerate(records):
        for right in records[left_index + 1:]:
            distance = math.dist((left.x, left.y), (right.x, right.y))
            if distance > near_distance_m:
                continue

            relation = SemanticRelation()
            relation.header.stamp = coerce_time(stamp)
            relation.header.frame_id = frame_id
            relation.subject_id = left.entity_id
            relation.predicate = "near"
            relation.object_id = right.entity_id
            relation.confidence = max(0.0, min(0.95, 1.0 - distance / near_distance_m))
            relations.append(relation)
    return relations
