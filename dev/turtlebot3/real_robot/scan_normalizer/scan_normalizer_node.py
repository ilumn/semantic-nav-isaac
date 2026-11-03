"""Republish LDS scans with stable geometry for consumers that require it.

Some real LDS-02/LD08 scans vary their angular metadata by a ray or two between
rotations. Costmaps tolerate that, but slam_toolbox's Karto layer registers the
first scan geometry and rejects later scans that do not match exactly.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from scan_processing import nearest_circular_ranges


class ScanNormalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_normalizer_node")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_slam")
        self.declare_parameter("target_count", 360)
        self.declare_parameter("frame_id", "")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._target_count = int(self.get_parameter("target_count").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._angle_increment = (2.0 * math.pi) / float(self._target_count)
        self._angle_max = self._angle_increment * float(self._target_count - 1)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._publisher = self.create_publisher(LaserScan, output_topic, qos)
        self.create_subscription(LaserScan, input_topic, self._scan_cb, qos)

        self.get_logger().info(
            "Normalizing %s -> %s with %d fixed rays"
            % (input_topic, output_topic, self._target_count)
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        if self._frame_id:
            out.header.frame_id = self._frame_id

        out.angle_min = 0.0
        out.angle_max = self._angle_max
        out.angle_increment = self._angle_increment
        out.time_increment = float(msg.scan_time) / float(self._target_count)
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        ranges = nearest_circular_ranges(
            list(msg.ranges),
            float(msg.angle_min),
            float(msg.angle_increment),
            self._target_count,
        )
        out.ranges = ranges
        out.intensities = []

        self._publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanNormalizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
