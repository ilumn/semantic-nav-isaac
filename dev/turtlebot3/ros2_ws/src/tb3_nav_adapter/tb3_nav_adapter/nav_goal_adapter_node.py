#!/usr/bin/env python3
"""
nav_goal_adapter_node.py — Stage-5 nav goal adapter ROS 2 node.

Subscribes to SemanticQueryResult from Stage 4, computes a safe approach
pose, optionally transforms it from base_link to map via TF, and publishes
a PoseStamped suitable for Nav2.

Subscribed topics
-----------------
  /semantic_query_node/selected_target   tb3_query/SemanticQueryResult

Published topics
----------------
  ~/goal_pose                            geometry_msgs/PoseStamped

Parameters  (see config/nav_goal_adapter.yaml)
----------
  approach_distance       float   Stand-off from target (m)
  min_standoff_distance   float   Reject targets closer than this (m)
  input_topic             str     SemanticQueryResult topic
  output_topic            str     PoseStamped goal topic
  target_frame            str     Desired output frame (default "map")
  tf_timeout              float   TF lookup timeout (s)
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PointStamped transform

from tb3_nav_adapter.goal_adapter_core import (
    compute_approach_pose,
    compute_approach_pose_global,
    yaw_to_quaternion,
)

try:
    from tb3_query.msg import SemanticQueryResult
except ImportError:
    SemanticQueryResult = None


class NavGoalAdapterNode(Node):

    def __init__(self) -> None:
        super().__init__("nav_goal_adapter_node")

        if SemanticQueryResult is None:
            self.get_logger().fatal(
                "tb3_query/msg/SemanticQueryResult not available. "
                "Did you source install/setup.bash after building tb3_query?"
            )
            raise RuntimeError("SemanticQueryResult message not found")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("approach_distance", 0.5)
        self.declare_parameter("min_standoff_distance", 0.3)
        self.declare_parameter("input_topic", "/semantic_query_node/selected_target")
        self.declare_parameter("output_topic", "~/goal_pose")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout", 0.5)
        self.declare_parameter("prefer_query_anchor", True)
        self.declare_parameter("robot_base_frame", "base_link")

        self._approach_dist = self.get_parameter("approach_distance").value
        self._min_standoff  = self.get_parameter("min_standoff_distance").value
        in_topic            = self.get_parameter("input_topic").value
        out_topic           = self.get_parameter("output_topic").value
        self._target_frame  = self.get_parameter("target_frame").value
        self._tf_timeout    = self.get_parameter("tf_timeout").value
        self._prefer_anchor = bool(self.get_parameter("prefer_query_anchor").value)
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value)

        # ── TF ────────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Sub / Pub ─────────────────────────────────────────────────────
        self.create_subscription(
            SemanticQueryResult, in_topic, self._query_cb, reliable_qos
        )
        self._goal_pub = self.create_publisher(PoseStamped, out_topic, reliable_qos)

        self.get_logger().info(
            "NavGoalAdapterNode ready  approach=%.2fm  standoff=%.2fm  target_frame=%s  prefer_anchor=%s"
            % (self._approach_dist, self._min_standoff, self._target_frame, self._prefer_anchor)
        )

    # ── Callback ──────────────────────────────────────────────────────────

    @staticmethod
    def _yaw_from_quaternion(z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _lookup_robot_xy(self, frame_id: str) -> tuple[float | None, float | None]:
        if frame_id == self._robot_base_frame:
            return 0.0, 0.0
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id,
                self._robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None, None
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )

    def _query_cb(self, msg: SemanticQueryResult) -> None:
        if not msg.success:
            self.get_logger().debug(
                "Query unsuccessful, skipping: %s" % msg.status_message
            )
            return

        tx = msg.position.x
        ty = msg.position.y
        source_frame = msg.frame_id or "base_link"

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = source_frame
        goal.pose.position.z = 0.0
        used_anchor = bool(msg.has_anchor and self._prefer_anchor)
        yaw = 0.0

        if used_anchor:
            goal.pose = msg.anchor_pose
            goal.pose.position.z = 0.0
            if goal.pose.orientation.w == 0.0 and goal.pose.orientation.z == 0.0:
                goal.pose.orientation.w = 1.0
            yaw = self._yaw_from_quaternion(
                goal.pose.orientation.z,
                goal.pose.orientation.w,
            )
            self.get_logger().info(
                "[%s] %s → using anchor %s (%s)"
                % (msg.semantic_name, msg.object_id, msg.anchor_id, msg.anchor_type)
            )
        else:
            if source_frame == self._robot_base_frame:
                result = compute_approach_pose(
                    tx, ty, self._approach_dist, self._min_standoff
                )
            else:
                robot_x, robot_y = self._lookup_robot_xy(source_frame)
                if robot_x is None or robot_y is None:
                    self.get_logger().warn(
                        "Robot pose unavailable in %s, skipping non-anchor goal for %s"
                        % (source_frame, msg.object_id)
                    )
                    return
                result = compute_approach_pose_global(
                    tx,
                    ty,
                    robot_x,
                    robot_y,
                    self._approach_dist,
                    self._min_standoff,
                )

            if result is None:
                self.get_logger().warn(
                    "Target %s too close (%.2fm), skipping goal" % (msg.object_id, (tx**2 + ty**2)**0.5)
                )
                return

            gx, gy, yaw = result
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            goal.pose.position.x = gx
            goal.pose.position.y = gy
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw

        # ── TF transform to target_frame ──────────────────────────────
        if self._target_frame != source_frame:
            try:
                goal = self._tf_buffer.transform(
                    goal,
                    self._target_frame,
                    timeout=Duration(seconds=self._tf_timeout),
                )
                self.get_logger().info(
                    "[%s] %s → approach (%.2f, %.2f) in %s  yaw=%.1f°"
                    % (
                        msg.semantic_name,
                        msg.object_id,
                        goal.pose.position.x,
                        goal.pose.position.y,
                        self._target_frame,
                        yaw * 57.2958,
                    )
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warn(
                    "TF %s→%s unavailable (%s), publishing in %s"
                    % (source_frame, self._target_frame, e, source_frame)
                )
        else:
            self.get_logger().info(
                "[%s] %s → approach (%.2f, %.2f) in %s  yaw=%.1f°"
                % (
                    msg.semantic_name,
                    msg.object_id,
                    goal.pose.position.x, goal.pose.position.y,
                    source_frame,
                    yaw * 57.2958,
                )
            )

        self._goal_pub.publish(goal)

        # TODO: optionally call NavigateToPose action client here


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalAdapterNode()
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
