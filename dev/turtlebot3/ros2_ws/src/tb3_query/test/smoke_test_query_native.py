#!/usr/bin/env python3
"""
smoke_test_query_native.py — Native semantic-map smoke test for tb3_query.

Launches a small ROS-only validation loop:

- publishes a synthetic /semantic_map/state
- sends target-only and relational commands
- checks /semantic_query_node/selected_target

Requires: semantic_query_node running in memory_mode=semantic_map_state.
"""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

try:
    from tb3_query.msg import SemanticQueryResult
    from tb3_semantic_map_msgs.msg import (
        SemanticAnchor,
        SemanticEntity,
        SemanticMapState,
        SemanticRelation,
    )
except ImportError:
    print("ERROR: required ROS messages not found. Did you source install/setup.bash?")
    sys.exit(1)


TIMEOUT_SEC = 5.0

TEST_CASES: list[dict] = [
    {"command": "go to the person", "expect_success": True, "expect_id": "person_0"},
    {"command": "go to the person next to the table", "expect_success": True, "expect_id": "person_1"},
    {"command": "go to the person left of the table", "expect_success": True, "expect_id": "person_1"},
    {"command": "go to the stop sign near the table", "expect_success": False, "expect_id": ""},
]


def _pose(x: float, y: float, z: float = 0.0) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def _entity(entity_id: str, semantic_name: str, detector_label: str, x: float, y: float, confidence: float) -> SemanticEntity:
    msg = SemanticEntity()
    msg.header.frame_id = "base_link"
    msg.entity_id = entity_id
    msg.semantic_name = semantic_name
    msg.detector_label = detector_label
    msg.pose = _pose(x, y)
    msg.extent.x = 0.5
    msg.extent.y = 0.5
    msg.extent.z = 1.0
    msg.confidence = confidence
    msg.observation_count = 1
    return msg


def _relation(subject: str, predicate: str, object_id: str, confidence: float) -> SemanticRelation:
    msg = SemanticRelation()
    msg.header.frame_id = "base_link"
    msg.subject_id = subject
    msg.predicate = predicate
    msg.object_id = object_id
    msg.confidence = confidence
    return msg


def _anchor(anchor_id: str, target_id: str, x: float, y: float, confidence: float) -> SemanticAnchor:
    msg = SemanticAnchor()
    msg.header.frame_id = "base_link"
    msg.anchor_id = anchor_id
    msg.anchor_type = "inspection"
    msg.target_id = target_id
    msg.pose = _pose(x, y)
    msg.confidence = confidence
    return msg


def _test_state() -> SemanticMapState:
    msg = SemanticMapState()
    msg.header.frame_id = "base_link"
    msg.source = "smoke_test/native_state"
    msg.refinement_active = False
    msg.entities = [
        _entity("person_0", "person", "person", 1.0, 0.0, 0.70),
        _entity("person_1", "person", "person", 3.0, 0.0, 0.90),
        _entity("table_0", "table", "bench", 3.1, 0.1, 0.95),
    ]
    msg.relations = [
        _relation("person_0", "near", "table_0", 0.30),
        _relation("person_1", "near", "table_0", 0.85),
    ]
    msg.anchors = [
        _anchor("anchor_person_1", "person_1", 2.7, -0.2, 0.88),
    ]
    return msg


class NativeSmokeTestNode(Node):

    def __init__(self) -> None:
        super().__init__("smoke_test_query_native")

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._cmd_pub = self.create_publisher(String, "/semantic_query_node/command", qos)
        self._state_pub = self.create_publisher(SemanticMapState, "/semantic_map/state", qos)
        self._result: SemanticQueryResult | None = None
        self.create_subscription(
            SemanticQueryResult,
            "/semantic_query_node/selected_target",
            self._result_cb,
            qos,
        )

    def _result_cb(self, msg: SemanticQueryResult) -> None:
        self._result = msg

    def publish_state(self) -> None:
        self._state_pub.publish(_test_state())

    def send_and_wait(self, command: str) -> SemanticQueryResult | None:
        self._result = None
        msg = String()
        msg.data = command
        self._cmd_pub.publish(msg)

        deadline = time.time() + TIMEOUT_SEC
        while self._result is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._result


def main() -> None:
    rclpy.init()
    node = NativeSmokeTestNode()

    for _ in range(3):
        node.publish_state()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.2)

    passed = 0
    failed = 0

    print("=" * 64)
    print("  tb3_query native semantic-map smoke test")
    print("=" * 64)

    for case in TEST_CASES:
        command = case["command"]
        expect_success = case["expect_success"]
        expect_id = case["expect_id"]

        node.publish_state()
        result = node.send_and_wait(command)
        print(f"\nCommand: {command}")

        if result is None:
            print(f"  FAIL — no response within {TIMEOUT_SEC:.1f}s")
            failed += 1
            continue

        ok = result.success == expect_success
        if expect_success and expect_id:
            ok = ok and result.object_id == expect_id

        if ok:
            if result.success:
                print(
                    "  PASS — %s -> %s at (%.2f, %.2f) anchor=%s"
                    % (
                        result.semantic_name,
                        result.object_id,
                        result.position.x,
                        result.position.y,
                        result.has_anchor,
                    )
                )
            else:
                print(f"  PASS — correctly rejected: {result.status_message}")
            passed += 1
        else:
            print(
                "  FAIL — success=%s object_id=%s status=%s"
                % (result.success, result.object_id, result.status_message)
            )
            failed += 1

    print("\n" + "=" * 64)
    print("  Results: %d passed, %d failed" % (passed, failed))
    print("=" * 64)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
