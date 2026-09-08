#!/usr/bin/env python3
"""
semantic_query_node.py — Stage-4 semantic query ROS 2 node.

Resolves target-only and simple relational text commands against the
current semantic memory state and publishes a SemanticQueryResult with
the selected target object.

Subscribed topics
-----------------
  /semantic_memory_node/objects     vision_msgs/Detection3DArray
  ~/command                         std_msgs/String

Published topics
----------------
  ~/selected_target                 tb3_query/SemanticQueryResult
  ~/query_status                    std_msgs/String   (human-readable debug)

Parameters  (see config/semantic_query.yaml)
----------
  memory_topic          str     Legacy memory state topic
  memory_mode           str     detection3d or semantic_map_state
  semantic_map_topic    str     Native semantic map topic
  command_topic         str     Input command topic
  output_topic          str     Selected target topic
  status_topic          str     Debug status topic
  semantic_targets_file str     Path to semantic_targets.yaml
  output_frame          str     Frame for output positions
"""

from __future__ import annotations

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener

from ament_index_python.packages import get_package_share_directory

from tb3_query.query_core import (
    MemoryAnchor,
    MemoryObject,
    MemoryRelation,
    attach_anchor,
    load_target_mapping,
    parse_command,
    select_best_anchor,
    select_relational_target,
    select_target,
)

try:
    from tb3_query.msg import SemanticQueryResult
except ImportError:
    SemanticQueryResult = None


class SemanticQueryNode(Node):

    def __init__(self) -> None:
        super().__init__("semantic_query_node")

        if SemanticQueryResult is None:
            self.get_logger().fatal(
                "tb3_query/msg/SemanticQueryResult not available. "
                "Did you source install/setup.bash after building?"
            )
            raise RuntimeError("SemanticQueryResult message not found")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("memory_topic", "/semantic_memory_node/objects")
        self.declare_parameter("memory_mode", "semantic_map_state")
        self.declare_parameter("semantic_map_topic", "/semantic_map/state")
        self.declare_parameter("command_topic", "~/command")
        self.declare_parameter("output_topic", "~/selected_target")
        self.declare_parameter("status_topic", "~/query_status")
        self.declare_parameter("semantic_targets_file", "")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")

        mem_topic    = self.get_parameter("memory_topic").value
        memory_mode  = self.get_parameter("memory_mode").value
        semantic_map_topic = self.get_parameter("semantic_map_topic").value
        cmd_topic    = self.get_parameter("command_topic").value
        out_topic    = self.get_parameter("output_topic").value
        status_topic = self.get_parameter("status_topic").value
        targets_file = self.get_parameter("semantic_targets_file").value
        self._out_frame = self.get_parameter("output_frame").value
        self._memory_mode = memory_mode
        self._robot_base_frame = self.get_parameter("robot_base_frame").value

        # ── Load semantic mapping ─────────────────────────────────────────
        if not targets_file:
            pkg_share = get_package_share_directory("tb3_frontier_exploration")
            targets_file = os.path.join(pkg_share, "config", "semantic_targets.yaml")

        self._sem2det, self._det2sem = load_target_mapping(targets_file)
        self._known_targets = set(self._sem2det.keys())
        self._memory_frame = self._out_frame
        self.get_logger().info(
            "Loaded %d semantic targets: %s"
            % (len(self._known_targets), sorted(self._known_targets))
        )

        # ── State: latest memory snapshot ─────────────────────────────────
        self._memory_objects: list[MemoryObject] = []
        self._memory_relations: list[MemoryRelation] = []
        self._memory_anchors: list[MemoryAnchor] = []
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        if self._memory_mode == "semantic_map_state":
            try:
                from tb3_semantic_map_msgs.msg import SemanticMapState
            except ImportError:
                self.get_logger().fatal(
                    "memory_mode=semantic_map_state but tb3_semantic_map_msgs is unavailable. "
                    "Build the workspace and source install/setup.bash."
                )
                raise
            self.create_subscription(
                SemanticMapState, semantic_map_topic, self._semantic_map_cb, reliable_qos
            )
        else:
            self.create_subscription(
                Detection3DArray, mem_topic, self._memory_cb, reliable_qos
            )
        self.create_subscription(
            String, cmd_topic, self._command_cb, reliable_qos
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._result_pub = self.create_publisher(
            SemanticQueryResult, out_topic, reliable_qos
        )
        self._status_pub = self.create_publisher(
            String, status_topic, reliable_qos
        )

        self.get_logger().info(
            "SemanticQueryNode ready  memory_mode=%s  frame=%s" % (self._memory_mode, self._out_frame)
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _memory_cb(self, msg: Detection3DArray) -> None:
        objs: list[MemoryObject] = []
        self._memory_relations = []
        self._memory_anchors = []
        self._memory_frame = msg.header.frame_id or self._out_frame or "base_link"
        for det in msg.detections:
            if not det.results:
                continue
            objs.append(MemoryObject(
                object_id=det.id,
                detector_label=det.results[0].hypothesis.class_id,
                semantic_name=self._det2sem.get(det.results[0].hypothesis.class_id, ""),
                x=det.bbox.center.position.x,
                y=det.bbox.center.position.y,
                confidence=det.results[0].hypothesis.score,
            ))
        self._memory_objects = objs

    @staticmethod
    def _yaw_from_quaternion(z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _semantic_map_cb(self, msg) -> None:
        objs: list[MemoryObject] = []
        anchors: list[MemoryAnchor] = []
        self._memory_frame = msg.header.frame_id or self._out_frame or "map"
        for entity in msg.entities:
            objs.append(MemoryObject(
                object_id=entity.entity_id,
                detector_label=entity.detector_label,
                semantic_name=entity.semantic_name,
                x=entity.pose.position.x,
                y=entity.pose.position.y,
                confidence=entity.confidence,
            ))
        self._memory_relations = [
            MemoryRelation(
                subject_id=relation.subject_id,
                predicate=relation.predicate,
                object_id=relation.object_id,
                confidence=relation.confidence,
            )
            for relation in msg.relations
        ]
        for anchor in msg.anchors:
            anchors.append(MemoryAnchor(
                anchor_id=anchor.anchor_id,
                anchor_type=anchor.anchor_type,
                target_id=anchor.target_id,
                x=anchor.pose.position.x,
                y=anchor.pose.position.y,
                yaw=self._yaw_from_quaternion(
                    anchor.pose.orientation.z,
                    anchor.pose.orientation.w if anchor.pose.orientation.w != 0.0 else 1.0,
                ),
                confidence=anchor.confidence,
            ))
        self._memory_objects = objs
        self._memory_anchors = anchors

    def _lookup_robot_xy(self, frame_id: str) -> tuple[float, float]:
        source_frame = frame_id or self._out_frame or self._robot_base_frame
        if source_frame == self._robot_base_frame:
            return 0.0, 0.0
        try:
            transform = self._tf_buffer.lookup_transform(
                source_frame,
                self._robot_base_frame,
                Time(),
            )
        except Exception:
            return 0.0, 0.0
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )

    def _command_cb(self, msg: String) -> None:
        raw = msg.data.strip()
        self.get_logger().info("Command received: '%s'" % raw)

        command = parse_command(raw, self._known_targets)

        if command is None:
            self._publish_failure(
                raw, "", "",
                f"unsupported semantic target in: '{raw}' "
                f"(known: {sorted(self._known_targets)})"
            )
            return

        semantic_name = command.target_semantic_name
        detector_label = self._sem2det.get(semantic_name, "")
        if not detector_label:
            self._publish_failure(
                raw, semantic_name, "",
                f"no detector_label mapping for '{semantic_name}'"
            )
            return

        robot_x, robot_y = self._lookup_robot_xy(self._memory_frame or self._out_frame or self._robot_base_frame)
        if command.has_relation and self._memory_mode != "semantic_map_state":
            self._publish_failure(
                raw,
                semantic_name,
                detector_label,
                "relational queries require memory_mode=semantic_map_state",
            )
            return

        if command.has_relation:
            reference_label = self._sem2det.get(command.reference_semantic_name, "")
            if not reference_label:
                self._publish_failure(
                    raw,
                    semantic_name,
                    detector_label,
                    f"no detector_label mapping for '{command.reference_semantic_name}'",
                )
                return
            result = select_relational_target(
                self._memory_objects,
                self._memory_relations,
                semantic_name,
                detector_label,
                command.reference_semantic_name,
                reference_label,
                command.relation_predicate,
                raw,
                robot_x=robot_x,
                robot_y=robot_y,
            )
        else:
            result = select_target(
                self._memory_objects,
                semantic_name,
                detector_label,
                raw,
                robot_x=robot_x,
                robot_y=robot_y,
            )
        if result.success and self._memory_mode == "semantic_map_state":
            result = attach_anchor(
                result,
                select_best_anchor(self._memory_anchors, result.object_id),
            )
        result_frame = self._memory_frame or self._out_frame or "base_link"

        out = SemanticQueryResult()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = result_frame
        out.success = result.success
        out.query_text = result.query_text
        out.semantic_name = result.semantic_name
        out.detector_label = result.detector_label
        out.object_id = result.object_id
        out.position = Point(x=result.x, y=result.y, z=0.0)
        out.frame_id = result_frame
        out.confidence = float(result.confidence)
        out.has_anchor = bool(result.has_anchor)
        out.anchor_id = result.anchor_id
        out.anchor_type = result.anchor_type
        out.anchor_pose.position.x = result.anchor_x
        out.anchor_pose.position.y = result.anchor_y
        out.anchor_pose.position.z = 0.0
        out.anchor_pose.orientation.z = math.sin(result.anchor_yaw / 2.0)
        out.anchor_pose.orientation.w = math.cos(result.anchor_yaw / 2.0)
        out.anchor_confidence = float(result.anchor_confidence)
        out.status_message = result.status_message

        self._result_pub.publish(out)

        status = String()
        status.data = result.status_message
        self._status_pub.publish(status)

        if result.success:
            self.get_logger().info(
                "Query OK: %s → %s at (%.2f, %.2f)"
                % (semantic_name, result.object_id, result.x, result.y)
            )
        else:
            self.get_logger().warn("Query FAILED: %s" % result.status_message)

    def _publish_failure(
        self, raw: str, semantic_name: str, detector_label: str, msg: str
    ) -> None:
        out = SemanticQueryResult()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._memory_frame or self._out_frame or "base_link"
        out.success = False
        out.query_text = raw
        out.semantic_name = semantic_name
        out.detector_label = detector_label
        out.frame_id = self._memory_frame or self._out_frame or "base_link"
        out.has_anchor = False
        out.anchor_pose.orientation.w = 1.0
        out.status_message = msg
        self._result_pub.publish(out)

        status = String()
        status.data = msg
        self._status_pub.publish(status)
        self.get_logger().warn("Query FAILED: %s" % msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
