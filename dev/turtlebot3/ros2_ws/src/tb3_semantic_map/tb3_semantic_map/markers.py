from __future__ import annotations

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


CLASS_COLORS = {
    "person": ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.9),
    "bench": ColorRGBA(r=0.8, g=0.6, b=0.2, a=0.9),
    "stop sign": ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.9),
}
DEFAULT_COLOR = ColorRGBA(r=0.4, g=0.6, b=0.9, a=0.9)


class MarkerBuilder:
    """Build RViz markers for semantic entities and delete stale ids."""

    def __init__(self, sphere_radius: float, text_offset_z: float) -> None:
        self._sphere_radius = sphere_radius
        self._text_offset_z = text_offset_z

    def build(self, state) -> MarkerArray:
        markers = MarkerArray()
        stamp = state.header.stamp
        frame_id = state.header.frame_id
        marker_id = 0

        delete_all = Marker()
        delete_all.header.stamp = stamp
        delete_all.header.frame_id = frame_id
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for entity in state.entities:
            color = CLASS_COLORS.get(entity.detector_label, DEFAULT_COLOR)

            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = frame_id
            sphere.ns = "semantic_entities"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = entity.pose
            sphere.scale.x = self._sphere_radius * 2.0
            sphere.scale.y = self._sphere_radius * 2.0
            sphere.scale.z = self._sphere_radius * 2.0
            sphere.color = color
            markers.markers.append(sphere)
            marker_id += 1

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = frame_id
            text.ns = "semantic_entity_text"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = entity.pose.position.x
            text.pose.position.y = entity.pose.position.y
            text.pose.position.z = entity.pose.position.z + self._text_offset_z
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            if entity.semantic_name and entity.semantic_name != entity.detector_label:
                text.text = "%s (%s/%s)" % (
                    entity.entity_id,
                    entity.semantic_name,
                    entity.detector_label,
                )
            else:
                text.text = "%s (%s)" % (
                    entity.entity_id,
                    entity.semantic_name or entity.detector_label,
                )
            markers.markers.append(text)
            marker_id += 1

        for place in state.places:
            place_marker = Marker()
            place_marker.header.stamp = stamp
            place_marker.header.frame_id = frame_id
            place_marker.ns = "semantic_places"
            place_marker.id = marker_id
            place_marker.type = Marker.CYLINDER
            place_marker.action = Marker.ADD
            place_marker.pose = place.anchor_pose
            place_marker.scale.x = max(0.3, place.extent.x)
            place_marker.scale.y = max(0.3, place.extent.y)
            place_marker.scale.z = 0.02
            place_marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.25)
            markers.markers.append(place_marker)
            marker_id += 1

        for anchor in state.anchors:
            anchor_marker = Marker()
            anchor_marker.header.stamp = stamp
            anchor_marker.header.frame_id = frame_id
            anchor_marker.ns = "semantic_anchors"
            anchor_marker.id = marker_id
            anchor_marker.type = Marker.ARROW
            anchor_marker.action = Marker.ADD
            anchor_marker.pose = anchor.pose
            anchor_marker.scale.x = 0.35
            anchor_marker.scale.y = 0.06
            anchor_marker.scale.z = 0.06
            anchor_marker.color = ColorRGBA(r=1.0, g=0.8, b=0.1, a=0.9)
            markers.markers.append(anchor_marker)
            marker_id += 1
        return markers
