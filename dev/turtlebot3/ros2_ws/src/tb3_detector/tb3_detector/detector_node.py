#!/usr/bin/env python3
"""
detector_node.py  —  ROS 2 node wrapper for Stage-1 Locate Anything detection.

Subscribed topics
-----------------
  /camera/image_raw          sensor_msgs/Image      (raw BGR / RGB camera)
  /camera/camera_info        sensor_msgs/CameraInfo (optional; logged but unused in Stage 1)

Published topics
----------------
  ~/detections               vision_msgs/Detection2DArray
      Contains one Detection2D per detected object:
        - results[0].hypothesis.class_id  = class label (str)
        - results[0].hypothesis.score     = 1.0 when the model returned the box
        - bbox.center.x / .y / size_x / size_y  = bounding box
        - id                              = track_id or "" if none

  ~/debug_image              sensor_msgs/Image   (only if ~publish_debug_image: true)
      Original image with Locate Anything bounding-box overlays.

Parameters
----------
  model_id                str   Hugging Face model ID or local snapshot directory
  model_revision          str   Pinned Hugging Face commit
  device                  str   "cuda:0"
  class_filter            list  Open-vocabulary categories to locate
  generation_mode         str   "fast" | "slow" | "hybrid"
  max_new_tokens          int   Generation limit
  local_files_only        bool  Refuse implicit downloads at node startup
  publish_debug_image     bool  true
  image_topic             str   "/camera/image_raw"
  camera_info_topic       str   "/camera/camera_info"
  detections_topic        str   "~/detections"
  debug_image_topic       str   "~/debug_image"
  queue_size              int   5
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

import cv2

try:
    from cv_bridge import CvBridge
except ImportError as e:
    raise ImportError(
        "cv_bridge not found. Install ros-humble-cv-bridge.\n"
        f"Original error: {e}"
    )

# Local detector logic (same package)
from tb3_detector.detector_core import DetectorCore


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_detection2d_array(detections: list[dict], stamp, frame_id: str) -> Detection2DArray:
    """Convert list of DetectorCore dicts → vision_msgs/Detection2DArray."""
    msg = Detection2DArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    for d in detections:
        det = Detection2D()
        det.header.stamp = stamp
        det.header.frame_id = frame_id

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = d["label"]
        hyp.hypothesis.score = float(d["conf"])
        det.results.append(hyp)

        x1, y1, x2, y2 = d["bbox_xyxy"]
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = x2 - x1
        h = y2 - y1

        det.bbox.center.position.x = cx
        det.bbox.center.position.y = cy
        det.bbox.size_x = w
        det.bbox.size_y = h

        # track_id as string id field (placeholder for Stage-2 memory)
        if d.get("track_id") is not None:
            det.id = str(d["track_id"])

        msg.detections.append(det)

    return msg


def _draw_detections(bgr_image, detections: list[dict]):
    """Draw bounding boxes + labels on a copy of bgr_image (for debug only)."""
    vis = bgr_image.copy()
    for d in detections:
        x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
        label = d["label"]
        tid = d.get("track_id")
        tag = label
        if tid is not None:
            tag += f" #{tid}"
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 50), 2)
        cv2.putText(vis, tag, (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 50), 2)
    return vis


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class DetectorNode(Node):
    """
    Stage-1 detector node.

    Lifecycle:
        __init__  → declare params, load model, create pubs/subs.
    The node then runs fully via subscription callbacks; no timer loop needed.

    TODO (Stage 2 – localizer):
        Subscribe to /camera/depth/image_raw and /camera/camera_info.
        Project each bounding-box centre into 3D using the camera intrinsics + depth.
        Publish a PoseStamped or PointStamped per detection for the memory module.
    """

    def __init__(self) -> None:
        super().__init__("detector_node")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("model_id", "nvidia/LocateAnything-3B")
        self.declare_parameter(
            "model_revision", "c32291ca5e996f5a7a485845b4f57a233936bba0"
        )
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("class_filter", ["bench", "person", "stop sign"])
        self.declare_parameter("generation_mode", "hybrid")
        self.declare_parameter("max_new_tokens", 2048)
        self.declare_parameter("local_files_only", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("detections_topic", "~/detections")
        self.declare_parameter("debug_image_topic", "~/debug_image")
        self.declare_parameter("queue_size", 5)

        model_id       = self.get_parameter("model_id").get_parameter_value().string_value
        model_revision = self.get_parameter("model_revision").get_parameter_value().string_value
        device         = self.get_parameter("device").get_parameter_value().string_value

        # ROS cannot infer a string-array type from a bare empty YAML list.
        try:
            raw_filter = self.get_parameter("class_filter").get_parameter_value().string_array_value
        except Exception:
            raw_filter = []
        generation_mode = self.get_parameter("generation_mode").value
        max_new_tokens = self.get_parameter("max_new_tokens").value
        local_files_only = self.get_parameter("local_files_only").value
        self._pub_debug = self.get_parameter("publish_debug_image").get_parameter_value().bool_value
        image_topic    = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic     = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        det_topic      = self.get_parameter("detections_topic").get_parameter_value().string_value
        dbg_topic      = self.get_parameter("debug_image_topic").get_parameter_value().string_value
        queue          = self.get_parameter("queue_size").get_parameter_value().integer_value

        class_filter = [c for c in raw_filter if c.strip()]
        if not class_filter:
            raise ValueError("class_filter must contain at least one Locate Anything query")

        # ── Inference core ─────────────────────────────────────────────────
        self._core = DetectorCore(
            model_id=model_id,
            model_revision=model_revision,
            class_filter=class_filter,
            device=device,
            generation_mode=generation_mode,
            max_new_tokens=max_new_tokens,
            local_files_only=local_files_only,
        )
        try:
            self._core.load()
        except Exception as exc:
            self.get_logger().error("DetectorCore failed to load: %s" % exc)
            self.get_logger().error(
                "► Did you complete the STOP HERE checkpoint? "
                "Check README.md → 'Model download checkpoint'."
            )
            raise

        # ── cv_bridge ──────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── QoS ────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=queue,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=queue,
        )

        # ── Subscriptions ──────────────────────────────────────────────────
        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, sensor_qos
        )
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._camera_info_callback, sensor_qos
        )
        self._latest_camera_info: CameraInfo | None = None

        # ── Publishers ─────────────────────────────────────────────────────
        self._det_pub = self.create_publisher(Detection2DArray, det_topic, reliable_qos)
        if self._pub_debug:
            self._dbg_pub = self.create_publisher(Image, dbg_topic, reliable_qos)
        else:
            self._dbg_pub = None

        self.get_logger().info(
            "detector_node ready. "
            "image_topic=%s  class_filter=%s  device=%s  generation_mode=%s"
            % (image_topic, class_filter, device, generation_mode)
        )

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Cache latest camera info.
        TODO (Stage 2): pass intrinsics to localizer for 3-D projection."""
        self._latest_camera_info = msg

    def _image_callback(self, msg: Image) -> None:
        """Main callback: convert → infer → publish."""
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning("cv_bridge conversion failed: %s" % e)
            return

        # ── Run inference ───────────────────────────────────────────────
        try:
            detections = self._core.infer(bgr)
        except Exception as e:
            self.get_logger().error("Inference error: %s" % e)
            return

        stamp = msg.header.stamp
        frame = msg.header.frame_id

        # ── Publish detections ──────────────────────────────────────────
        det_msg = _make_detection2d_array(detections, stamp, frame)
        self._det_pub.publish(det_msg)

        if detections:
            labels = [d["label"] for d in detections]
            self.get_logger().debug("Detected: %s" % labels)

        # ── Publish debug image ─────────────────────────────────────────
        if self._dbg_pub is not None:
            vis = _draw_detections(bgr, detections) if detections else bgr
            try:
                dbg_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
                dbg_msg.header = msg.header
                self._dbg_pub.publish(dbg_msg)
            except Exception as e:
                self.get_logger().warning("Debug image publish failed: %s" % e)

        # TODO (Stage 2 – localizer):
        #   If self._latest_camera_info is not None:
        #       for det in detections:
        #           point_3d = project_to_3d(det["bbox_xyxy"], depth_image, self._latest_camera_info)
        #           localizer.publish(point_3d, det["label"])

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
