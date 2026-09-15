from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import Detection3DArray
from ament_index_python.packages import PackageNotFoundError

from tb3_semantic_map_msgs.msg import SemanticMapState

from .compat import state_to_detection3d_array
from .detector_bridge import DetectorBridge
from .frame_sampler import FrameSampler
from .grounding import ObservationGrounder, yaw_from_quaternion
from .markers import MarkerBuilder
from .memory_core import LiveSemanticMemory
from .memory_core import SemanticObservation
from .state_builder import build_semantic_map_state
from .target_mapping import default_targets_file, load_target_mapping, semantic_name_for_label

import cv2

try:
    from cv_bridge import CvBridge
except ImportError as e:
    raise ImportError(
        "cv_bridge not found. Install ros-humble-cv-bridge.\n"
        f"Original error: {e}"
    )


class SemanticMapNode(Node):

    def __init__(self) -> None:
        super().__init__("semantic_map_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("semantic_targets_file", "")
        self.declare_parameter("state_topic", "/semantic_map/state")
        self.declare_parameter("compat_topic", "/semantic_map/compat_objects")
        self.declare_parameter("markers_topic", "/semantic_map/markers")
        self.declare_parameter("status_topic", "/semantic_map/status")
        self.declare_parameter("debug_image_topic", "/semantic_map/debug_image")
        self.declare_parameter("refined_state_topic", "/semantic_map/refined_state")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("detector_backend", "locate_anything")
        self.declare_parameter("model_id", "nvidia/LocateAnything-3B")
        self.declare_parameter(
            "model_revision", "c32291ca5e996f5a7a485845b4f57a233936bba0"
        )
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("generation_mode", "hybrid")
        self.declare_parameter("max_new_tokens", 2048)
        self.declare_parameter("local_files_only", True)
        self.declare_parameter("class_filter", ["__from_targets__"])
        self.declare_parameter("frame_sample_interval_sec", 0.5)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("camera_hfov_deg", 62.2)
        self.declare_parameter("camera_base_tx", 0.0)
        self.declare_parameter("camera_base_ty", 0.0)
        self.declare_parameter("camera_base_yaw_deg", 0.0)
        self.declare_parameter("scan_window_half", 5)
        self.declare_parameter("min_valid_range", 0.12)
        self.declare_parameter("max_valid_range", 8.0)
        self.declare_parameter("bearing_sigma_deg", 2.0)
        self.declare_parameter("range_sigma_ratio", 0.08)
        self.declare_parameter("min_range_sigma_m", 0.05)
        self.declare_parameter("tf_timeout_sec", 0.3)
        self.declare_parameter("entity_match_distance_m", 1.0)
        self.declare_parameter("position_smoothing_alpha", 0.3)
        self.declare_parameter("stale_timeout", 5.0)
        self.declare_parameter("remove_timeout", 30.0)
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("enable_refinement_enrichment", True)
        self.declare_parameter("refinement_match_distance_m", 1.0)
        self.declare_parameter("refinement_min_confidence", 0.55)
        self.declare_parameter("sphere_radius", 0.18)
        self.declare_parameter("text_offset_z", 0.45)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        scan_topic = self.get_parameter("scan_topic").value
        map_topic = self.get_parameter("map_topic").value
        semantic_targets_file = self.get_parameter("semantic_targets_file").value
        state_topic = self.get_parameter("state_topic").value
        compat_topic = self.get_parameter("compat_topic").value
        markers_topic = self.get_parameter("markers_topic").value
        status_topic = self.get_parameter("status_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value
        refined_state_topic = self.get_parameter("refined_state_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self._detector_backend = self.get_parameter("detector_backend").value
        model_id = self.get_parameter("model_id").value
        model_revision = self.get_parameter("model_revision").value
        device = self.get_parameter("device").value
        generation_mode = self.get_parameter("generation_mode").value
        max_new_tokens = int(self.get_parameter("max_new_tokens").value)
        local_files_only = bool(self.get_parameter("local_files_only").value)
        configured_class_filter = [item for item in self.get_parameter("class_filter").value if item]
        self._publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        camera_hfov_deg = float(self.get_parameter("camera_hfov_deg").value)
        camera_base_tx = float(self.get_parameter("camera_base_tx").value)
        camera_base_ty = float(self.get_parameter("camera_base_ty").value)
        camera_base_yaw_deg = float(self.get_parameter("camera_base_yaw_deg").value)
        scan_window_half = int(self.get_parameter("scan_window_half").value)
        min_valid_range = float(self.get_parameter("min_valid_range").value)
        max_valid_range = float(self.get_parameter("max_valid_range").value)
        bearing_sigma_deg = float(self.get_parameter("bearing_sigma_deg").value)
        range_sigma_ratio = float(self.get_parameter("range_sigma_ratio").value)
        min_range_sigma_m = float(self.get_parameter("min_range_sigma_m").value)
        self._tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self._match_distance = float(self.get_parameter("entity_match_distance_m").value)
        self._alpha = float(self.get_parameter("position_smoothing_alpha").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._remove_timeout = float(self.get_parameter("remove_timeout").value)
        publish_rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self._enable_refinement_enrichment = bool(self.get_parameter("enable_refinement_enrichment").value)
        self._refinement_match_distance_m = float(self.get_parameter("refinement_match_distance_m").value)
        self._refinement_min_confidence = float(self.get_parameter("refinement_min_confidence").value)
        sample_interval = float(self.get_parameter("frame_sample_interval_sec").value)
        sphere_radius = float(self.get_parameter("sphere_radius").value)
        text_offset_z = float(self.get_parameter("text_offset_z").value)

        if not semantic_targets_file:
            try:
                semantic_targets_file = default_targets_file()
            except PackageNotFoundError:
                semantic_targets_file = ""
        if semantic_targets_file:
            self._sem2det, self._det2sem = load_target_mapping(semantic_targets_file)
        else:
            self._sem2det, self._det2sem = {}, {}

        if configured_class_filter == ["__from_targets__"] or not configured_class_filter:
            self._class_filter = list(dict.fromkeys(self._sem2det.values())) if self._sem2det else []
        else:
            self._class_filter = configured_class_filter

        if not self._class_filter and self._sem2det:
            self._class_filter = list(dict.fromkeys(self._sem2det.values()))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._sampler = FrameSampler(sample_interval)
        self._grounder = ObservationGrounder(
            camera_hfov_deg=camera_hfov_deg,
            scan_window_half=scan_window_half,
            min_valid_range=min_valid_range,
            max_valid_range=max_valid_range,
            camera_base_tx=camera_base_tx,
            camera_base_ty=camera_base_ty,
            camera_base_yaw_deg=camera_base_yaw_deg,
            bearing_sigma_deg=bearing_sigma_deg,
            range_sigma_ratio=range_sigma_ratio,
            min_range_sigma_m=min_range_sigma_m,
        )
        self._memory = LiveSemanticMemory(
            match_distance_threshold=self._match_distance,
            position_smoothing_alpha=self._alpha,
            stale_timeout=self._stale_timeout,
            remove_timeout=self._remove_timeout,
        )
        self._marker_builder = MarkerBuilder(sphere_radius=sphere_radius, text_offset_z=text_offset_z)
        self._bridge = CvBridge()
        self._detector = DetectorBridge(
            model_id=model_id,
            model_revision=model_revision,
            class_filter=self._class_filter or None,
            device=device,
            generation_mode=generation_mode,
            max_new_tokens=max_new_tokens,
            local_files_only=local_files_only,
        )
        self._detector.load()

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._latest_image: Optional[Image] = None
        self._latest_sampled_image: Optional[Image] = None
        self._latest_camera_info: Optional[CameraInfo] = None
        self._latest_scan: Optional[LaserScan] = None
        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_refined_state: Optional[SemanticMapState] = None
        self._sampled_frame_count = 0
        self._last_detection_count = 0
        self._last_grounded_count = 0

        self.create_subscription(Image, image_topic, self._image_cb, sensor_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, sensor_qos)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, sensor_qos)
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)
        if self._enable_refinement_enrichment:
            self.create_subscription(SemanticMapState, refined_state_topic, self._refined_state_cb, reliable_qos)

        self._state_pub = self.create_publisher(SemanticMapState, state_topic, reliable_qos)
        self._compat_pub = self.create_publisher(Detection3DArray, compat_topic, reliable_qos)
        self._markers_pub = self.create_publisher(MarkerArray, markers_topic, reliable_qos)
        self._status_pub = self.create_publisher(String, status_topic, reliable_qos)
        self._debug_pub = self.create_publisher(Image, debug_image_topic, reliable_qos) if self._publish_debug_image else None

        self.create_timer(1.0 / publish_rate, self._publish_outputs)

        self.get_logger().info(
            "SemanticMapNode ready  detector=%s  frame=%s  sample_interval=%.2fs  "
            "match_distance=%.2fm"
            % (
                self._detector_backend,
                self._target_frame,
                sample_interval,
                self._match_distance,
            )
        )

    def _image_cb(self, msg: Image) -> None:
        self._latest_image = msg
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self._sampler.should_sample(stamp_sec):
            self._latest_sampled_image = msg
            self._sampled_frame_count += 1
            self._process_sampled_image(msg, stamp_sec)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _refined_state_cb(self, msg: SemanticMapState) -> None:
        self._latest_refined_state = msg

    def _build_state(self) -> SemanticMapState:
        return build_semantic_map_state(
            self._memory,
            self.get_clock().now().to_msg(),
            self._target_frame,
            refined_state=self._latest_refined_state if self._enable_refinement_enrichment else None,
            refinement_match_distance_m=self._refinement_match_distance_m,
            refinement_min_confidence=self._refinement_min_confidence,
        )

    def _lookup_robot_pose(self) -> tuple[Optional[float], Optional[float], Optional[float]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._robot_base_frame,
                Time(),
            )
        except Exception:
            return None, None, None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            float(yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)),
        )

    def _point_inside_map(self, x: float, y: float) -> bool:
        if self._latest_map is None:
            return True
        info = self._latest_map.info
        max_x = info.origin.position.x + info.width * info.resolution
        max_y = info.origin.position.y + info.height * info.resolution
        return info.origin.position.x <= x <= max_x and info.origin.position.y <= y <= max_y

    def _draw_debug_image(self, bgr_image, detections: list[dict]):
        vis = bgr_image.copy()
        for detection in detections:
            x1, y1, x2, y2 = [int(v) for v in detection["bbox_xyxy"]]
            label = detection["label"]
            conf = detection["conf"]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 50), 2)
            cv2.putText(
                vis,
                f"{label} {conf:.2f}",
                (x1, max(y1 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 200, 50),
                2,
            )
        return vis

    def _process_sampled_image(self, msg: Image, stamp_sec: float) -> None:
        if self._latest_scan is None:
            return
        robot_x, robot_y, robot_yaw = self._lookup_robot_pose()
        if robot_x is None or robot_y is None or robot_yaw is None:
            return

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning("cv_bridge conversion failed: %s" % exc)
            return

        try:
            detections = self._detector.infer(bgr)
        except Exception as exc:
            self.get_logger().error("Semantic map detector inference failed: %s" % exc)
            return

        self._last_detection_count = len(detections)
        grounded_count = 0
        for detection in detections:
            semantic_name = semantic_name_for_label(detection["label"], self._det2sem)
            grounded = self._grounder.ground_detection(
                detection=detection,
                image_width=int(msg.width),
                scan_ranges=list(self._latest_scan.ranges),
                scan_angle_min=float(self._latest_scan.angle_min),
                scan_angle_max=float(self._latest_scan.angle_max),
                scan_angle_increment=float(self._latest_scan.angle_increment),
                robot_tx=robot_x,
                robot_ty=robot_y,
                robot_yaw=robot_yaw,
                semantic_name=semantic_name,
            )
            if grounded is None or not self._point_inside_map(grounded.map_x, grounded.map_y):
                continue
            grounded_count += 1
            self._memory.update(
                SemanticObservation(
                    semantic_name=grounded.semantic_name,
                    detector_label=grounded.detector_label,
                    confidence=grounded.confidence,
                    x=grounded.map_x,
                    y=grounded.map_y,
                    z=grounded.map_z,
                    timestamp_sec=stamp_sec,
                    covariance_xy=grounded.covariance_xy,
                    aliases=[grounded.detector_label],
                    provenance=["live_ros2_grounded"],
                )
            )

        self._last_grounded_count = grounded_count

        if self._debug_pub is not None:
            vis = self._draw_debug_image(bgr, detections)
            debug_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            debug_msg.header = msg.header
            self._debug_pub.publish(debug_msg)

    def _publish_outputs(self) -> None:
        now = self.get_clock().now()
        self._memory.age(float(now.nanoseconds) / 1e9)
        state = self._build_state()
        compat = state_to_detection3d_array(state)
        markers = self._marker_builder.build(state)

        self._state_pub.publish(state)
        self._compat_pub.publish(compat)
        self._markers_pub.publish(markers)

        status = String()
        status.data = (
            "semantic_map heartbeat | sampled_frames=%d | detections=%d | grounded=%d | "
            "camera_info=%s | scan=%s | map=%s | refined_state=%s | "
            "entities=%d | class_filter=%s"
            % (
                self._sampled_frame_count,
                self._last_detection_count,
                self._last_grounded_count,
                self._latest_camera_info is not None,
                self._latest_scan is not None,
                self._latest_map is not None,
                self._latest_refined_state is not None,
                self._memory.size,
                ",".join(self._class_filter) if self._class_filter else "all",
            )
        )
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
