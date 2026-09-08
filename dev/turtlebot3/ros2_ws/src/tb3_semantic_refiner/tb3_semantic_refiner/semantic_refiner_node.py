from __future__ import annotations

import json
import math
import threading
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import ColorRGBA, Header, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from tb3_semantic_map_msgs.msg import (
    SemanticAnchor,
    SemanticEntity,
    SemanticMapState,
    SemanticPlace,
    SemanticRelation,
)

try:
    from cv_bridge import CvBridge
except ImportError as e:
    raise ImportError(
        "cv_bridge not found. Install ros-humble-cv-bridge.\n"
        f"Original error: {e}"
    )

from .aligned_scene import AlignedScene, load_aligned_scene
from .bundle_io import BundleWriter
from .bundle_quality import BundleReadiness, assess_bundle_readiness
from .bundle_core import BufferedFrame, FrameBundleBuffer
from .frame_sampler import FrameSampler
from .job_core import RefinerJobRecord
from .pointcloud import build_xyzrgb_cloud, pack_rgb_uint32
from .worker_core import WorkerConfig, cleanup_old_jobs, run_semantic_nav_memory_job


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SemanticRefinerNode(Node):

    def __init__(self) -> None:
        super().__init__("semantic_refiner_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("status_topic", "/semantic_map/refiner_status")
        self.declare_parameter("output_markers_topic", "/semantic_map/colmap_markers")
        self.declare_parameter("output_state_topic", "/semantic_map/refined_state")
        self.declare_parameter("output_pointcloud_topic", "/semantic_map/colmap_points")
        self.declare_parameter("output_support_pointcloud_topic", "/semantic_map/colmap_support_points")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("bundle_root", "/tmp/tb3_semantic_refiner_jobs")
        self.declare_parameter("max_completed_jobs", 5)
        self.declare_parameter("max_buffered_frames", 180)
        self.declare_parameter("frame_sample_interval_sec", 0.35)
        self.declare_parameter("frame_sample_min_translation_m", 0.05)
        self.declare_parameter("frame_sample_min_yaw_rad", 0.08)
        self.declare_parameter("worker_enabled", False)
        self.declare_parameter("auto_run_when_buffer_ready", False)
        self.declare_parameter("min_buffered_frames_for_job", 28)
        self.declare_parameter("min_pose_frames_for_job", 18)
        self.declare_parameter("min_bundle_translation_span_m", 0.75)
        self.declare_parameter("min_bundle_path_length_m", 1.5)
        self.declare_parameter("min_new_frames_for_job", 14)
        self.declare_parameter("worker_timeout_sec", 300.0)
        self.declare_parameter("worker_python_executable", "python3")
        self.declare_parameter("worker_max_keyframes", 128)
        self.declare_parameter("worker_max_image_size", 1440)
        self.declare_parameter("worker_yolo_model_size", "")
        self.declare_parameter("worker_yolo_model_name", "")
        self.declare_parameter("worker_yolo_confidence", 0.18)
        self.declare_parameter("worker_prompt_preset", "")
        self.declare_parameter("worker_scene_profile", "general")
        self.declare_parameter("worker_prompt_vocabulary", [])

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        map_topic = self.get_parameter("map_topic").value
        status_topic = self.get_parameter("status_topic").value
        markers_topic = self.get_parameter("output_markers_topic").value
        state_topic = self.get_parameter("output_state_topic").value
        pointcloud_topic = self.get_parameter("output_pointcloud_topic").value
        support_pointcloud_topic = self.get_parameter("output_support_pointcloud_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self._bundle_root = self.get_parameter("bundle_root").value
        self._max_completed_jobs = int(self.get_parameter("max_completed_jobs").value)
        self._worker_enabled = bool(self.get_parameter("worker_enabled").value)
        self._auto_run_when_buffer_ready = bool(self.get_parameter("auto_run_when_buffer_ready").value)
        self._min_buffered_frames_for_job = int(self.get_parameter("min_buffered_frames_for_job").value)
        self._min_pose_frames_for_job = int(self.get_parameter("min_pose_frames_for_job").value)
        self._min_bundle_translation_span_m = float(
            self.get_parameter("min_bundle_translation_span_m").value
        )
        self._min_bundle_path_length_m = float(self.get_parameter("min_bundle_path_length_m").value)
        self._min_new_frames_for_job = int(self.get_parameter("min_new_frames_for_job").value)
        self._worker_timeout_sec = float(self.get_parameter("worker_timeout_sec").value)
        max_frames = int(self.get_parameter("max_buffered_frames").value)
        sample_interval = float(self.get_parameter("frame_sample_interval_sec").value)
        min_translation = float(self.get_parameter("frame_sample_min_translation_m").value)
        min_yaw = float(self.get_parameter("frame_sample_min_yaw_rad").value)

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        self._latest_camera_info: Optional[CameraInfo] = None
        self._latest_map: Optional[OccupancyGrid] = None
        self._buffer = FrameBundleBuffer(max_frames)
        self._bundle_writer = BundleWriter(self._bundle_root, max_frames)
        self._sampler = FrameSampler(
            sample_interval,
            min_translation_m=min_translation,
            min_yaw_delta_rad=min_yaw,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._bridge = CvBridge()
        self._job_lock = threading.Lock()
        self._job_thread: Optional[threading.Thread] = None
        self._saved_frame_count = 0
        self._active_job = False
        self._last_job: Optional[RefinerJobRecord] = None
        self._last_job_saved_frame_count = 0
        self._startup_completed_job_ids = self._existing_completed_job_ids()
        self._startup_accepted_job_ids = self._existing_accepted_job_ids()
        self._latest_bundle_readiness = BundleReadiness(
            total_frames=0,
            pose_frames=0,
            translation_span_m=0.0,
            path_length_m=0.0,
            yaw_span_rad=0.0,
            ready=False,
            reason="waiting_for_frames",
        )
        self._overlay_scene: Optional[AlignedScene] = None
        self._overlay_job_id = ""

        self.create_subscription(Image, image_topic, self._image_cb, sensor_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, sensor_qos)
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)

        self._status_pub = self.create_publisher(String, status_topic, reliable_qos)
        self._markers_pub = self.create_publisher(MarkerArray, markers_topic, reliable_qos)
        self._state_pub = self.create_publisher(SemanticMapState, state_topic, reliable_qos)
        self._pointcloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, map_qos)
        self._support_pointcloud_pub = self.create_publisher(PointCloud2, support_pointcloud_topic, map_qos)

        self.create_timer(1.0, self._publish_overlay)
        self.create_timer(1.0, self._publish_status)
        self.create_timer(5.0, self._maybe_run_job)

        self.get_logger().info(
            "SemanticRefinerNode ready  frame=%s  base=%s  bundle_root=%s  "
            "worker_enabled=%s  worker_python=%s"
            % (
                self._target_frame,
                self._robot_base_frame,
                self._bundle_root,
                self._worker_enabled,
                self.get_parameter("worker_python_executable").value,
            )
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

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
            float(_yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)),
        )

    def _image_cb(self, msg: Image) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        fx = fy = cx = cy = None
        if self._latest_camera_info is not None and len(self._latest_camera_info.k) >= 9:
            fx = float(self._latest_camera_info.k[0])
            fy = float(self._latest_camera_info.k[4])
            cx = float(self._latest_camera_info.k[2])
            cy = float(self._latest_camera_info.k[5])

        robot_x, robot_y, robot_yaw = self._lookup_robot_pose()
        if not self._sampler.should_sample(stamp_sec, robot_x, robot_y, robot_yaw):
            return

        buffered = BufferedFrame(
            stamp_sec=stamp_sec,
            width=int(msg.width),
            height=int(msg.height),
            frame_id=msg.header.frame_id,
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
        )
        self._buffer.add(buffered)

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._bundle_writer.write_frame(
                frame_index=self._saved_frame_count,
                bgr_image=bgr,
                metadata={
                    "stamp_sec": buffered.stamp_sec,
                    "width": buffered.width,
                    "height": buffered.height,
                    "frame_id": buffered.frame_id,
                    "robot_x": buffered.robot_x,
                    "robot_y": buffered.robot_y,
                    "robot_yaw": buffered.robot_yaw,
                    "fx": buffered.fx,
                    "fy": buffered.fy,
                    "cx": buffered.cx,
                    "cy": buffered.cy,
                },
            )
            self._saved_frame_count += 1
        except Exception as exc:
            self.get_logger().warning("semantic refiner failed to persist sampled frame: %s" % exc)

        self._latest_bundle_readiness = self._assess_bundle_readiness()

    def _worker_config(self) -> WorkerConfig:
        try:
            prompt_items = list(
                self.get_parameter("worker_prompt_vocabulary").get_parameter_value().string_array_value
            )
        except Exception:
            prompt_items = []
        return WorkerConfig(
            python_executable=str(self.get_parameter("worker_python_executable").value),
            frame_sample_fps=0.0,
            max_keyframes=int(self.get_parameter("worker_max_keyframes").value),
            max_image_size=int(self.get_parameter("worker_max_image_size").value),
            yolo_model_size=str(self.get_parameter("worker_yolo_model_size").value),
            yolo_model_name=str(self.get_parameter("worker_yolo_model_name").value),
            yolo_confidence=float(self.get_parameter("worker_yolo_confidence").value),
            prompt_preset=str(self.get_parameter("worker_prompt_preset").value),
            scene_profile=str(self.get_parameter("worker_scene_profile").value),
            prompt_vocabulary=[str(item) for item in prompt_items if str(item)],
        )

    def _job_worker_main(self) -> None:
        try:
            job = run_semantic_nav_memory_job(
                bundle_root=self._bundle_root,
                worker_timeout_sec=self._worker_timeout_sec,
                worker_config=self._worker_config(),
            )
            cleanup_old_jobs(self._bundle_root, self._max_completed_jobs)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"semantic refiner worker thread failed: {exc}")
            job = None
        with self._job_lock:
            self._last_job = job
            self._active_job = False
            self._job_thread = None

    def _maybe_run_job(self) -> None:
        if not self._worker_enabled or not self._auto_run_when_buffer_ready:
            return
        snapshot = self._buffer.snapshot()
        self._latest_bundle_readiness = self._assess_bundle_readiness(snapshot)
        if len(snapshot) < self._min_buffered_frames_for_job:
            return
        if not self._latest_bundle_readiness.ready:
            return
        if self._saved_frame_count - self._last_job_saved_frame_count < self._min_new_frames_for_job:
            return
        with self._job_lock:
            if self._active_job:
                return
            self._active_job = True
            self._last_job_saved_frame_count = self._saved_frame_count
            self._job_thread = threading.Thread(target=self._job_worker_main, daemon=True)
            self._job_thread.start()

    def _assess_bundle_readiness(
        self,
        frames: Optional[list[BufferedFrame]] = None,
    ) -> BundleReadiness:
        return assess_bundle_readiness(
            self._buffer.snapshot() if frames is None else frames,
            min_pose_frames=self._min_pose_frames_for_job,
            min_translation_span_m=self._min_bundle_translation_span_m,
            min_path_length_m=self._min_bundle_path_length_m,
        )

    def _latest_accepted_job_dir(self) -> Optional[Path]:
        latest = self._latest_completed_job()
        if latest is None:
            return None
        job_dir, payload = latest
        if not bool(payload.get("accepted_refinement", False)):
            return None
        return job_dir

    def _latest_completed_job(self) -> Optional[tuple[Path, dict]]:
        jobs_dir = Path(self._bundle_root) / "jobs"
        if not jobs_dir.exists():
            return None
        candidates: list[tuple[float, Path, dict]] = []
        for job_dir in jobs_dir.iterdir():
            completed = self._completed_job_payload(job_dir)
            if completed is None:
                continue
            if job_dir.name in self._startup_completed_job_ids:
                continue
            candidates.append((completed[0], job_dir, completed[1]))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        return (candidates[0][1], candidates[0][2])

    def _existing_completed_job_ids(self) -> set[str]:
        jobs_dir = Path(self._bundle_root) / "jobs"
        if not jobs_dir.exists():
            return set()
        existing: set[str] = set()
        for job_dir in jobs_dir.iterdir():
            if self._completed_job_payload(job_dir) is not None:
                existing.add(job_dir.name)
        return existing

    def _existing_accepted_job_ids(self) -> set[str]:
        jobs_dir = Path(self._bundle_root) / "jobs"
        if not jobs_dir.exists():
            return set()
        existing: set[str] = set()
        for job_dir in jobs_dir.iterdir():
            if self._accepted_job_payload(job_dir) is not None:
                existing.add(job_dir.name)
        return existing

    def _accepted_job_payload(self, job_dir: Path) -> Optional[tuple[float, dict]]:
        completed = self._completed_job_payload(job_dir)
        if completed is None:
            return None
        if not bool(completed[1].get("accepted_refinement", False)):
            return None
        return completed

    def _completed_job_payload(self, job_dir: Path) -> Optional[tuple[float, dict]]:
        job_path = job_dir / "job.json"
        if not job_dir.is_dir() or not job_path.exists():
            return None
        try:
            payload = json.loads(job_path.read_text(encoding="utf-8"))
        except Exception:
            return None
        if payload.get("status") != "completed":
            return None
        if str(payload.get("job_id", "")).startswith("bootstrap_overlay"):
            return None
        scene_graph_path = job_dir / "outputs" / "scene_graph.json"
        if scene_graph_path.exists():
            try:
                scene_graph = json.loads(scene_graph_path.read_text(encoding="utf-8"))
            except Exception:
                scene_graph = {}
            diagnostics = scene_graph.get("diagnostics", {})
            if isinstance(diagnostics, dict) and diagnostics.get("bootstrap_overlay", False):
                return None
        return (job_path.stat().st_mtime, payload)

    def _drop_overlay(self) -> None:
        had_overlay = self._overlay_scene is not None or bool(self._overlay_job_id)
        self._overlay_scene = None
        self._overlay_job_id = ""
        if had_overlay:
            self._clear_overlay()

    def _clear_overlay(self) -> None:
        delete_all = Marker()
        delete_all.header.frame_id = self._target_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        markers = MarkerArray()
        markers.markers.append(delete_all)
        self._markers_pub.publish(markers)
        self._pointcloud_pub.publish(self._empty_pointcloud())
        self._support_pointcloud_pub.publish(self._empty_pointcloud())

    def _empty_pointcloud(self) -> PointCloud2:
        header = self._cloud_header()
        return build_xyzrgb_cloud(header=header, points=[])

    def _cloud_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._target_frame
        return header

    def _build_overlay_cloud(self, scene: AlignedScene, use_support_points: bool = False) -> PointCloud2:
        source_points = scene.support_points if use_support_points else scene.sparse_points
        if use_support_points:
            point_iter = (
                (point.x, point.y, point.z, pack_rgb_uint32(*point.color_rgb))
                for point in source_points
            )
        else:
            point_iter = (
                (point.x, point.y, point.z, pack_rgb_uint32(*point.color_rgb))
                for point in source_points
            )
        return build_xyzrgb_cloud(header=self._cloud_header(), points=point_iter)

    def _build_overlay_state(self, scene: AlignedScene) -> SemanticMapState:
        stamp = self.get_clock().now().to_msg()
        state = SemanticMapState()
        state.header.stamp = stamp
        state.header.frame_id = self._target_frame
        state.source = "tb3_semantic_refiner/reference_only"
        state.refinement_active = True

        for record in scene.entities:
            entity = SemanticEntity()
            entity.header.stamp = stamp
            entity.header.frame_id = self._target_frame
            entity.entity_id = record.entity_id
            entity.semantic_name = record.semantic_name
            entity.detector_label = record.detector_label
            entity.place_id = record.place_id
            entity.state = record.state
            entity.aliases = list(record.aliases)
            entity.provenance = list(record.provenance)
            entity.pose.position.x = record.x
            entity.pose.position.y = record.y
            entity.pose.position.z = record.z
            entity.pose.orientation.w = 1.0
            entity.pose_covariance = [0.0] * 36
            entity.extent.x = record.extent_xyz[0]
            entity.extent.y = record.extent_xyz[1]
            entity.extent.z = record.extent_xyz[2]
            entity.confidence = float(record.confidence)
            entity.observation_count = int(record.observation_count)
            state.entities.append(entity)

        for record in scene.places:
            place = SemanticPlace()
            place.header.stamp = stamp
            place.header.frame_id = self._target_frame
            place.place_id = record.place_id
            place.place_type = record.place_type
            place.member_entity_ids = list(record.member_entity_ids)
            place.anchor_pose.position.x = record.x
            place.anchor_pose.position.y = record.y
            place.anchor_pose.position.z = record.z
            place.anchor_pose.orientation.w = 1.0
            place.extent.x = record.extent_xyz[0]
            place.extent.y = record.extent_xyz[1]
            place.extent.z = record.extent_xyz[2]
            place.confidence = float(record.confidence)
            state.places.append(place)

        for record in scene.relations:
            relation = SemanticRelation()
            relation.header.stamp = stamp
            relation.header.frame_id = self._target_frame
            relation.subject_id = record.subject_id
            relation.predicate = record.predicate
            relation.object_id = record.object_id
            relation.confidence = float(record.confidence)
            state.relations.append(relation)

        for record in scene.anchors:
            anchor = SemanticAnchor()
            anchor.header.stamp = stamp
            anchor.header.frame_id = self._target_frame
            anchor.anchor_id = record.anchor_id
            anchor.anchor_type = record.anchor_type
            anchor.target_id = record.target_id
            anchor.pose.position.x = record.x
            anchor.pose.position.y = record.y
            anchor.pose.position.z = record.z
            anchor.pose.orientation.z = math.sin(record.yaw_rad * 0.5)
            anchor.pose.orientation.w = math.cos(record.yaw_rad * 0.5)
            anchor.confidence = float(record.confidence)
            state.anchors.append(anchor)
        return state

    def _build_overlay_markers(self, scene: AlignedScene) -> MarkerArray:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        marker_id = 0

        delete_all = Marker()
        delete_all.header.stamp = stamp
        delete_all.header.frame_id = self._target_frame
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        entity_color = ColorRGBA(r=0.2, g=1.0, b=1.0, a=0.75)
        place_color = ColorRGBA(r=0.3, g=0.7, b=1.0, a=0.2)
        anchor_color = ColorRGBA(r=1.0, g=0.9, b=0.2, a=0.8)
        text_color = ColorRGBA(r=0.95, g=0.95, b=0.95, a=1.0)

        for entity in scene.entities:
            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = self._target_frame
            sphere.ns = "refined_entities"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = entity.x
            sphere.pose.position.y = entity.y
            sphere.pose.position.z = entity.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = max(0.12, entity.extent_xyz[0])
            sphere.scale.y = max(0.12, entity.extent_xyz[1])
            sphere.scale.z = max(0.12, entity.extent_xyz[2])
            sphere.color = entity_color
            markers.markers.append(sphere)
            marker_id += 1

            label = Marker()
            label.header.stamp = stamp
            label.header.frame_id = self._target_frame
            label.ns = "refined_entity_text"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = entity.x
            label.pose.position.y = entity.y
            label.pose.position.z = entity.z + max(0.2, entity.extent_xyz[2] * 0.75)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.14
            label.color = text_color
            label.text = f"{entity.entity_id} [{entity.semantic_name}]"
            markers.markers.append(label)
            marker_id += 1

        for place in scene.places:
            place_marker = Marker()
            place_marker.header.stamp = stamp
            place_marker.header.frame_id = self._target_frame
            place_marker.ns = "refined_places"
            place_marker.id = marker_id
            place_marker.type = Marker.CYLINDER
            place_marker.action = Marker.ADD
            place_marker.pose.position.x = place.x
            place_marker.pose.position.y = place.y
            place_marker.pose.position.z = place.z
            place_marker.pose.orientation.w = 1.0
            place_marker.scale.x = max(0.25, place.extent_xyz[0])
            place_marker.scale.y = max(0.25, place.extent_xyz[1])
            place_marker.scale.z = 0.03
            place_marker.color = place_color
            markers.markers.append(place_marker)
            marker_id += 1

        for anchor in scene.anchors:
            anchor_marker = Marker()
            anchor_marker.header.stamp = stamp
            anchor_marker.header.frame_id = self._target_frame
            anchor_marker.ns = "refined_anchors"
            anchor_marker.id = marker_id
            anchor_marker.type = Marker.ARROW
            anchor_marker.action = Marker.ADD
            anchor_marker.pose.position.x = anchor.x
            anchor_marker.pose.position.y = anchor.y
            anchor_marker.pose.position.z = anchor.z
            anchor_marker.pose.orientation.z = math.sin(anchor.yaw_rad * 0.5)
            anchor_marker.pose.orientation.w = math.cos(anchor.yaw_rad * 0.5)
            anchor_marker.scale.x = 0.35
            anchor_marker.scale.y = 0.05
            anchor_marker.scale.z = 0.05
            anchor_marker.color = anchor_color
            markers.markers.append(anchor_marker)
            marker_id += 1
        return markers

    def _publish_overlay(self) -> None:
        latest_job = self._latest_completed_job()
        if latest_job is None:
            self._drop_overlay()
            return

        accepted_job_dir, payload = latest_job
        if not bool(payload.get("accepted_refinement", False)):
            self._drop_overlay()
            return

        if accepted_job_dir.name != self._overlay_job_id:
            scene = load_aligned_scene(accepted_job_dir, require_accepted=True)
            if scene is None:
                self._drop_overlay()
                return
            self._overlay_scene = scene
            self._overlay_job_id = accepted_job_dir.name

        if self._overlay_scene is None:
            return
        self._markers_pub.publish(self._build_overlay_markers(self._overlay_scene))
        self._state_pub.publish(self._build_overlay_state(self._overlay_scene))
        self._pointcloud_pub.publish(self._build_overlay_cloud(self._overlay_scene))
        self._support_pointcloud_pub.publish(
            self._build_overlay_cloud(self._overlay_scene, use_support_points=True)
        )

    def _publish_status(self) -> None:
        status = String()
        last_job_text = "none"
        alignment_text = "none"
        with self._job_lock:
            last_job = self._last_job
            active_job = self._active_job
        if last_job is not None:
            last_job_text = f"{last_job.job_id}:{last_job.status}:{last_job.current_step}"
            if last_job.alignment:
                alignment_text = (
                    f"accepted={last_job.accepted_refinement},"
                    f"conf={last_job.alignment.get('confidence', 0.0)},"
                    f"rms={last_job.alignment.get('rms_error_m', 'n/a')}"
                )
        status.data = (
            "semantic_refiner heartbeat | buffered_frames=%d | saved_frames=%d | camera_info=%s | map=%s | "
            "worker_enabled=%s | active_job=%s | last_job=%s | alignment=%s | "
            "bundle_ready=%s | bundle_reason=%s | pose_frames=%d | bundle_span_m=%.3f | bundle_path_m=%.3f | bundle_yaw_rad=%.3f | "
            "overlay_job=%s | "
            "overlay_entities=%d | overlay_places=%d | overlay_anchors=%d | "
            "overlay_sparse_points=%d | overlay_support_points=%d | reference_only=true"
            % (
                len(self._buffer),
                self._saved_frame_count,
                self._latest_camera_info is not None,
                self._latest_map is not None,
                self._worker_enabled,
                active_job,
                last_job_text,
                alignment_text,
                self._latest_bundle_readiness.ready,
                self._latest_bundle_readiness.reason,
                self._latest_bundle_readiness.pose_frames,
                self._latest_bundle_readiness.translation_span_m,
                self._latest_bundle_readiness.path_length_m,
                self._latest_bundle_readiness.yaw_span_rad,
                self._overlay_job_id or "none",
                len(self._overlay_scene.entities) if self._overlay_scene is not None else 0,
                len(self._overlay_scene.places) if self._overlay_scene is not None else 0,
                len(self._overlay_scene.anchors) if self._overlay_scene is not None else 0,
                len(self._overlay_scene.sparse_points) if self._overlay_scene is not None else 0,
                len(self._overlay_scene.support_points) if self._overlay_scene is not None else 0,
            )
        )
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticRefinerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
