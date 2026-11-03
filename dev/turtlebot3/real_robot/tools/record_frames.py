#!/usr/bin/env python3
"""Save one camera frame and one detector frame per second to a run folder, for reviewing the vision model.

    python3 /mnt/data/semnav/tools/record_frames.py <output_dir> <seconds>
"""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main() -> None:
    out, seconds = sys.argv[1], float(sys.argv[2])
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("record_frames")
    latest = {}
    node.create_subscription(Image, "/camera/image_raw", lambda m: latest.__setitem__("raw", m), qos_profile_sensor_data)
    node.create_subscription(Image, "/semantic_map/live_debug_image", lambda m: latest.__setitem__("det", m), qos_profile_sensor_data)
    start = time.time()
    next_save = start
    index = 0
    while time.time() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() >= next_save and len(latest) == 2:
            for name, msg in latest.items():
                frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
                if msg.encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite(os.path.join(out, f"{index:04d}_{name}.jpg"), frame)
            index += 1
            next_save += 1.0
    print(f"saved {index} frame pairs to {out}")


if __name__ == "__main__":
    main()
