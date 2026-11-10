#!/usr/bin/env python3
"""Compressed copy of the detector image for remote viewers (Foxglove over Tailscale).

Encodes only while someone subscribes, at a reduced size and quality, so it keeps up over a
relayed link. Replaces image_transport republish, which encoded every frame at quality 95.
"""

import array

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data as sensor_qos
from sensor_msgs.msg import CompressedImage, Image

JPEG_QUALITY = 50
JPEG_WIDTH = 480


class DebugJpeg:
    def __init__(self) -> None:
        self.node = rclpy.create_node("debug_jpeg")
        self.pub = self.node.create_publisher(CompressedImage, "/semantic_map/live_debug_image/compressed", sensor_qos)
        self.node.create_subscription(Image, "/semantic_map/live_debug_image", self.on_image, sensor_qos)

    def on_image(self, msg: Image) -> None:
        if self.pub.get_subscription_count() == 0:
            return
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, (JPEG_WIDTH, JPEG_WIDTH * msg.height // msg.width), interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            self.pub.publish(CompressedImage(header=msg.header, format="jpeg", data=array.array("B", jpeg.tobytes())))


def main() -> None:
    rclpy.init()
    rclpy.spin(DebugJpeg().node)


if __name__ == "__main__":
    main()
