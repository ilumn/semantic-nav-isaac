#!/usr/bin/env python3
"""Rotate the upside-down camera 180 degrees at the source.

The USB camera has no hardware flip control, so v4l2_camera publishes on /camera_raw/* and
this node republishes on /camera/*, which is what everything else consumes. JPEG encoding
only happens while someone subscribes to the compressed topic.
"""

import array

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data as sensor_qos
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

JPEG_QUALITY = 70


class CameraFlip:
    def __init__(self) -> None:
        self.node = rclpy.create_node("camera_flip")
        self.image_pub = self.node.create_publisher(Image, "/camera/image_raw", sensor_qos)
        self.jpeg_pub = self.node.create_publisher(CompressedImage, "/camera/image_raw/compressed", sensor_qos)
        self.info_pub = self.node.create_publisher(CameraInfo, "/camera/camera_info", sensor_qos)
        self.node.create_subscription(Image, "/camera_raw/image_raw", self.on_image, sensor_qos)
        self.node.create_subscription(CameraInfo, "/camera_raw/camera_info", self.on_info, sensor_qos)

    def on_image(self, msg: Image) -> None:
        channels = msg.step // msg.width
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, channels)[::-1, ::-1]
        # array.array, not bytes: the generated setter checks bytes element by element in Python.
        msg.data = array.array("B", frame.tobytes())
        self.image_pub.publish(msg)
        if self.jpeg_pub.get_subscription_count() > 0:
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else frame
            ok, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                out = CompressedImage(header=msg.header, format="jpeg", data=array.array("B", jpeg.tobytes()))
                self.jpeg_pub.publish(out)

    def on_info(self, msg: CameraInfo) -> None:
        # A 180 degree rotation mirrors the principal point; zeros mean "uncalibrated".
        if msg.k[2] or msg.k[5]:
            msg.k[2], msg.k[5] = msg.width - msg.k[2], msg.height - msg.k[5]
            msg.p[2], msg.p[6] = msg.width - msg.p[2], msg.height - msg.p[6]
        self.info_pub.publish(msg)


def main() -> None:
    rclpy.init()
    rclpy.spin(CameraFlip().node)


if __name__ == "__main__":
    main()
