from __future__ import annotations

from tb3_semantic_map_msgs.msg import SemanticEntity, SemanticMapState

from .anchor_builder import build_anchors
from .enrichment import enrich_state_with_refinement
from .place_builder import build_places
from .relation_builder import build_relations

def record_to_semantic_entity(record, stamp, frame_id: str, entity_to_place: dict[str, str]) -> SemanticEntity:
    entity = SemanticEntity()
    entity.header.stamp = stamp
    entity.header.frame_id = frame_id
    entity.entity_id = record.entity_id
    entity.semantic_name = record.semantic_name
    entity.detector_label = record.detector_label
    entity.place_id = entity_to_place.get(record.entity_id, record.place_id)
    entity.state = record.state
    entity.aliases = list(record.aliases)
    entity.provenance = list(record.provenance)
    entity.pose.position.x = float(record.x)
    entity.pose.position.y = float(record.y)
    entity.pose.position.z = float(record.z)
    entity.pose.orientation.w = 1.0
    pose_covariance = [0.0] * 36
    pose_covariance[0] = float(record.covariance_xy[0])
    pose_covariance[1] = float(record.covariance_xy[1])
    pose_covariance[6] = float(record.covariance_xy[1])
    pose_covariance[7] = float(record.covariance_xy[2])
    pose_covariance[14] = 0.05
    pose_covariance[35] = 0.15
    entity.pose_covariance = pose_covariance
    entity.extent.x = float(record.extent_xyz[0])
    entity.extent.y = float(record.extent_xyz[1])
    entity.extent.z = float(record.extent_xyz[2])
    entity.confidence = float(record.confidence)
    entity.observation_count = int(record.observation_count)
    entity.last_seen.sec = int(record.last_seen)
    entity.last_seen.nanosec = int((record.last_seen - int(record.last_seen)) * 1e9)
    return entity


def build_semantic_map_state(
    memory,
    stamp,
    frame_id: str,
    refined_state=None,
    refinement_match_distance_m: float = 1.0,
    refinement_min_confidence: float = 0.55,
) -> SemanticMapState:
    records = memory.get_active_entities()
    places, entity_to_place = build_places(records, stamp, frame_id)
    relations = build_relations(records, stamp, frame_id)
    anchors = build_anchors(records, stamp, frame_id)

    state = SemanticMapState()
    state.header.stamp = stamp
    state.header.frame_id = frame_id
    state.source = "tb3_semantic_map/live"
    state.refinement_active = False
    state.entities = [record_to_semantic_entity(record, stamp, frame_id, entity_to_place) for record in records]
    state.places = places
    state.relations = relations
    state.anchors = anchors
    return enrich_state_with_refinement(
        state,
        refined_state=refined_state,
        match_distance_m=refinement_match_distance_m,
        min_confidence=refinement_min_confidence,
    )
