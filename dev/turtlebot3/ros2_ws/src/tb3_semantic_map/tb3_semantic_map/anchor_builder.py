from __future__ import annotations

import math

from tb3_semantic_map_msgs.msg import SemanticAnchor
from .ros_message_utils import coerce_time


def build_anchors(records: list, stamp, frame_id: str) -> list[SemanticAnchor]:
    anchors: list[SemanticAnchor] = []
    for index, record in enumerate(records, start=1):
        anchor = SemanticAnchor()
        anchor.header.stamp = coerce_time(stamp)
        anchor.header.frame_id = frame_id
        anchor.anchor_id = f"approach_anchor_{index:02d}"
        anchor.anchor_type = "approach"
        anchor.target_id = record.entity_id

        distance = max(record.extent_xyz[0], record.extent_xyz[1], 0.25) + 0.5
        anchor.pose.position.x = float(record.x - distance)
        anchor.pose.position.y = float(record.y)
        anchor.pose.position.z = float(record.z)
        anchor.pose.orientation.w = 1.0
        anchor.confidence = min(0.95, 0.5 + 0.3 * float(record.confidence))
        anchors.append(anchor)
    return anchors
