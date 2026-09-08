#!/usr/bin/env python3
"""
smoke_test_nav_goal_adapter_native.py — Native semantic-query smoke test for nav adapter.

Requires:
- nav_goal_adapter_node running
- static TF available for map -> base_link when testing world-frame non-anchor results
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

try:
    from tb3_query.msg import SemanticQueryResult
except ImportError:
    print("ERROR: tb3_query messages not found. Did you source install/setup.bash?")
    sys.exit(1)


TIMEOUT_SEC = 5.0


def _query_result(
    *,
    semantic_name: str,
    object_id: str,
    frame_id: str,
    x: float,
    y: float,
    has_anchor: bool = False,
    anchor_x: float = 0.0,
    anchor_y: float = 0.0,
    anchor_yaw: float = 0.0,
) -> SemanticQueryResult:
    msg = SemanticQueryResult()
    msg.success = True
    msg.semantic_name = semantic_name
    msg.detector_label = semantic_name
    msg.object_id = object_id
    msg.frame_id = frame_id
    msg.position.x = x
    msg.position.y = y
    msg.position.z = 0.0
    msg.confidence = 0.9
    msg.has_anchor = has_anchor
    if has_anchor:
        msg.anchor_id = f"anchor_{object_id}"
        msg.anchor_type = "inspection"
        msg.anchor_pose.position.x = anchor_x
        msg.anchor_pose.position.y = anchor_y
        msg.anchor_pose.orientation.z = math.sin(anchor_yaw / 2.0)
        msg.anchor_pose.orientation.w = math.cos(anchor_yaw / 2.0)
        msg.anchor_confidence = 0.85
    return msg


class NativeNavSmokeNode(Node):

    def __init__(self) -> None:
        super().__init__("smoke_test_nav_goal_adapter_native")

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._query_pub = self.create_publisher(SemanticQueryResult, "/semantic_query_node/selected_target", qos)
        self._goal: PoseStamped | None = None
        self.create_subscription(
            PoseStamped,
            "/nav_goal_adapter_node/goal_pose",
            self._goal_cb,
            qos,
        )

    def _goal_cb(self, msg: PoseStamped) -> None:
        self._goal = msg

    def send_and_wait(self, result: SemanticQueryResult) -> PoseStamped | None:
        self._goal = None
        self._query_pub.publish(result)
        deadline = time.time() + TIMEOUT_SEC
        while self._goal is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._goal


def main() -> None:
    rclpy.init()
    node = NativeNavSmokeNode()
    time.sleep(1.0)

    print("=" * 64)
    print("  tb3_nav_adapter native smoke test")
    print("=" * 64)

    base_result = _query_result(
        semantic_name="person",
        object_id="person_0",
        frame_id="base_link",
        x=2.0,
        y=0.0,
    )
    goal = node.send_and_wait(base_result)
    if goal is None:
        print("FAIL — no base_link goal")
        sys.exit(1)
    print("PASS — base_link non-anchor goal at (%.2f, %.2f)" % (goal.pose.position.x, goal.pose.position.y))

    anchor_result = _query_result(
        semantic_name="person",
        object_id="person_1",
        frame_id="map",
        x=3.0,
        y=0.0,
        has_anchor=True,
        anchor_x=2.7,
        anchor_y=-0.2,
        anchor_yaw=0.4,
    )
    goal = node.send_and_wait(anchor_result)
    if goal is None:
        print("FAIL — no anchor goal")
        sys.exit(1)
    print("PASS — anchor goal at (%.2f, %.2f) in %s" % (goal.pose.position.x, goal.pose.position.y, goal.header.frame_id))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
