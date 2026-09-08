from __future__ import annotations

from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from .ros_message_utils import coerce_header


def entity_to_detection3d(entity) -> Detection3D:
    msg = Detection3D()
    msg.header = coerce_header(entity.header)
    msg.id = entity.entity_id

    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = entity.detector_label
    hyp.hypothesis.score = float(entity.confidence)
    msg.results.append(hyp)

    msg.bbox.center.position.x = entity.pose.position.x
    msg.bbox.center.position.y = entity.pose.position.y
    msg.bbox.center.position.z = entity.pose.position.z
    return msg


def state_to_detection3d_array(state) -> Detection3DArray:
    out = Detection3DArray()
    out.header = coerce_header(state.header)
    out.detections = [entity_to_detection3d(entity) for entity in state.entities]
    return out
