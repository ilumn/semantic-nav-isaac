#!/usr/bin/env python3
"""Preflight checks for real TurtleBot3 semantic navigation deployment."""

from __future__ import annotations

import sys
import time
from typing import Dict, List

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros


class RealRobotPreflight(Node):
    def __init__(self) -> None:
        super().__init__("real_robot_preflight")

        self.declare_parameter("timeout_sec", 10.0)
        self.declare_parameter("topic_timeout_sec", 2.5)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "")

        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.topic_timeout_sec = float(self.get_parameter("topic_timeout_sec").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.last_seen: Dict[str, Time] = {}

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            lambda msg: self._mark_seen("scan"),
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            lambda msg: self._mark_seen("odom"),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_cb,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_cb,
            sensor_qos,
        )
        self.create_subscription(
            TFMessage, "/tf", lambda msg: self._mark_seen("tf"), sensor_qos)
        self.create_subscription(
            TFMessage, "/tf_static", lambda msg: self._mark_seen("tf_static"), static_tf_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _mark_seen(self, key: str) -> None:
        self.last_seen[key] = self.get_clock().now()

    def _image_cb(self, msg: Image) -> None:
        self._mark_seen("image")
        if not self.camera_frame and msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._mark_seen("camera_info")
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

    def _fresh(self, key: str) -> bool:
        stamp = self.last_seen.get(key)
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= self.topic_timeout_sec

    def _can_transform(self, target: str, source: str) -> bool:
        if not target or not source:
            return False
        try:
            return self.tf_buffer.can_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception:
            return False

    def failures(self) -> List[str]:
        failures: List[str] = []
        for key in ("scan", "odom", "image", "camera_info", "tf"):
            if not self._fresh(key):
                failures.append("%s missing/stale" % key)
        if "tf_static" not in self.last_seen:
            failures.append("tf_static missing")
        if not self._can_transform(self.map_frame, self.odom_frame):
            failures.append("%s->%s TF missing" % (self.map_frame, self.odom_frame))
        if not self._can_transform(self.odom_frame, self.base_frame):
            failures.append("%s->%s TF missing" % (self.odom_frame, self.base_frame))
        if self.camera_frame:
            if not self._can_transform(self.base_frame, self.camera_frame):
                failures.append("%s->%s TF missing" % (self.base_frame, self.camera_frame))
        else:
            failures.append("camera frame unknown")
        return failures


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealRobotPreflight()
    deadline = time.monotonic() + node.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.failures():
                print("PASS: real robot preflight checks passed")
                sys.exit(0)

        failures = node.failures()
        print("FAIL: real robot preflight checks did not pass")
        for failure in failures:
            print("- " + failure)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
