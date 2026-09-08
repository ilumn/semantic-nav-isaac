"""Read-only ROS graph and TF contract checker for NVIDIA Isaac Sim."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from .contract import (
    ContractViolation,
    RateWindow,
    TopicContract,
    TopicObservation,
    evaluate_contracts,
    required_tf_chain,
)


class IsaacContractChecker(Node):
    """Observe, validate, and report the simulator-facing ROS contract.

    The checker deliberately does not publish commands, clock, transforms, or
    sensor data. Its only publication is a JSON diagnostic status string.
    """

    def __init__(self) -> None:
        super().__init__("isaac_contract_checker")

        self._declare_parameters()
        self._started_monotonic = time.monotonic()
        self._startup_grace_sec = float(
            self.get_parameter("startup_grace_sec").value
        )
        self._report_period_sec = float(
            self.get_parameter("report_period_sec").value
        )
        self._shutdown_on_failure = bool(
            self.get_parameter("shutdown_on_failure").value
        )
        self._failure_reports_before_shutdown = max(
            1,
            int(self.get_parameter("failure_reports_before_shutdown").value),
        )
        self._consecutive_failure_reports = 0
        self._last_log_signature: tuple[str, ...] | None = None

        self._rates = RateWindow(
            float(self.get_parameter("rate_window_sec").value)
        )
        self._contracts = self._build_topic_contracts()
        self._message_types = self._build_message_types()

        self._sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._static_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Retain observer subscriptions without shadowing rclpy.node.Node's
        # private ``_subscriptions`` bookkeeping list.
        self._observer_subscriptions = []
        self._create_observer_subscriptions()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )
        self._tf_requirements = required_tf_chain(
            map_frame=str(self.get_parameter("map_frame").value),
            odom_frame=str(self.get_parameter("odom_frame").value),
            base_footprint_frame=str(
                self.get_parameter("base_footprint_frame").value
            ),
            base_frame=str(self.get_parameter("base_frame").value),
            scan_frame=str(self.get_parameter("scan_frame").value),
            camera_frame=str(self.get_parameter("camera_frame").value),
            camera_optical_frame=str(
                self.get_parameter("camera_optical_frame").value
            ),
            require_map_frame=bool(
                self.get_parameter("require_map_frame").value
            ),
            require_camera_optical_frame=bool(
                self.get_parameter("require_camera_optical_frame").value
            ),
        )

        status_topic = str(self.get_parameter("status_topic").value)
        self._status_publisher = self.create_publisher(
            String,
            status_topic,
            self._status_qos,
        )

        # A steady timer still fires when /clock is absent or simulation is paused.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._report_timer = self.create_timer(
            max(0.2, self._report_period_sec),
            self._report,
            clock=self._steady_clock,
        )
        self.get_logger().info(
            "Isaac ROS contract checker started; grace=%.1fs status=%s"
            % (self._startup_grace_sec, status_topic)
        )

    def _declare_parameters(self) -> None:
        topic_defaults = {
            "clock_topic": "/clock",
            "joint_states_topic": "/joint_states",
            "odom_topic": "/odom",
            "tf_topic": "/tf",
            "tf_static_topic": "/tf_static",
            "imu_topic": "/imu",
            "scan_topic": "/scan",
            "image_topic": "/camera/image_raw",
            "camera_info_topic": "/camera/camera_info",
            "cmd_vel_topic": "/cmd_vel",
            "status_topic": "/isaac_semantic_nav/contract_status",
        }
        for name, value in topic_defaults.items():
            self.declare_parameter(name, value)

        rate_defaults = {
            "clock_min_rate_hz": 10.0,
            "joint_states_min_rate_hz": 5.0,
            "odom_min_rate_hz": 10.0,
            "tf_min_rate_hz": 10.0,
            "imu_min_rate_hz": 20.0,
            "scan_min_rate_hz": 5.0,
            "image_min_rate_hz": 10.0,
            "camera_info_min_rate_hz": 1.0,
        }
        for name, value in rate_defaults.items():
            self.declare_parameter(name, value)

        self.declare_parameter("startup_grace_sec", 20.0)
        self.declare_parameter("report_period_sec", 2.0)
        self.declare_parameter("rate_window_sec", 5.0)
        self.declare_parameter("max_message_age_sec", 2.5)
        self.declare_parameter("shutdown_on_failure", False)
        self.declare_parameter("failure_reports_before_shutdown", 3)
        self.declare_parameter("require_joint_states", True)
        self.declare_parameter("require_imu", True)
        self.declare_parameter("require_tf_static_topic", True)

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_footprint_frame", "base_footprint")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_frame", "base_scan")
        self.declare_parameter("camera_frame", "camera_rgb_frame")
        self.declare_parameter(
            "camera_optical_frame",
            "camera_rgb_optical_frame",
        )
        self.declare_parameter("require_map_frame", True)
        self.declare_parameter("require_camera_optical_frame", True)
        self.declare_parameter("tf_timeout_sec", 0.05)

    def _topic(self, parameter_name: str) -> str:
        return str(self.get_parameter(parameter_name).value)

    def _rate(self, parameter_name: str) -> float:
        return max(0.0, float(self.get_parameter(parameter_name).value))

    def _build_topic_contracts(self) -> list[TopicContract]:
        max_age = max(
            0.1,
            float(self.get_parameter("max_message_age_sec").value),
        )

        contracts = [
            TopicContract(
                self._topic("clock_topic"),
                "rosgraph_msgs/msg/Clock",
                min_rate_hz=self._rate("clock_min_rate_hz"),
                max_age_sec=max_age,
            ),
            TopicContract(
                self._topic("odom_topic"),
                "nav_msgs/msg/Odometry",
                min_rate_hz=self._rate("odom_min_rate_hz"),
                max_age_sec=max_age,
            ),
            TopicContract(
                self._topic("tf_topic"),
                "tf2_msgs/msg/TFMessage",
                max_publishers=None,
                min_rate_hz=self._rate("tf_min_rate_hz"),
                max_age_sec=max_age,
            ),
            TopicContract(
                self._topic("scan_topic"),
                "sensor_msgs/msg/LaserScan",
                min_rate_hz=self._rate("scan_min_rate_hz"),
                max_age_sec=max_age,
            ),
            TopicContract(
                self._topic("image_topic"),
                "sensor_msgs/msg/Image",
                min_rate_hz=self._rate("image_min_rate_hz"),
                max_age_sec=max_age,
            ),
            TopicContract(
                self._topic("camera_info_topic"),
                "sensor_msgs/msg/CameraInfo",
                min_rate_hz=self._rate("camera_info_min_rate_hz"),
                max_age_sec=max_age,
            ),
            TopicContract(
                self._topic("cmd_vel_topic"),
                "geometry_msgs/msg/Twist",
                min_publishers=0,
                max_publishers=None,
                min_subscribers=1,
                require_sample=False,
            ),
        ]
        if bool(self.get_parameter("require_joint_states").value):
            contracts.append(
                TopicContract(
                    self._topic("joint_states_topic"),
                    "sensor_msgs/msg/JointState",
                    min_rate_hz=self._rate("joint_states_min_rate_hz"),
                    max_age_sec=max_age,
                )
            )
        if bool(self.get_parameter("require_imu").value):
            contracts.append(
                TopicContract(
                    self._topic("imu_topic"),
                    "sensor_msgs/msg/Imu",
                    min_rate_hz=self._rate("imu_min_rate_hz"),
                    max_age_sec=max_age,
                )
            )
        if bool(self.get_parameter("require_tf_static_topic").value):
            contracts.append(
                TopicContract(
                    self._topic("tf_static_topic"),
                    "tf2_msgs/msg/TFMessage",
                    max_publishers=None,
                    min_rate_hz=None,
                    max_age_sec=None,
                )
            )
        return contracts

    def _build_message_types(self) -> dict[str, Any]:
        return {
            self._topic("clock_topic"): ClockMessage,
            self._topic("joint_states_topic"): JointState,
            self._topic("odom_topic"): Odometry,
            self._topic("tf_topic"): TFMessage,
            self._topic("tf_static_topic"): TFMessage,
            self._topic("imu_topic"): Imu,
            self._topic("scan_topic"): LaserScan,
            self._topic("image_topic"): Image,
            self._topic("camera_info_topic"): CameraInfo,
            self._topic("cmd_vel_topic"): Twist,
        }

    def _create_observer_subscriptions(self) -> None:
        static_tf_topic = self._topic("tf_static_topic")
        for contract in self._contracts:
            if not contract.require_sample:
                continue
            message_type = self._message_types[contract.name]
            qos = self._static_tf_qos if contract.name == static_tf_topic else self._sensor_qos
            subscription = self.create_subscription(
                message_type,
                contract.name,
                lambda _message, topic=contract.name: self._rates.record(
                    topic,
                    time.monotonic(),
                ),
                qos,
            )
            self._observer_subscriptions.append(subscription)

    @staticmethod
    def _endpoint_types(endpoint_info: list[Any]) -> frozenset[str]:
        return frozenset(str(info.topic_type) for info in endpoint_info)

    def _observe_topics(self, now: float) -> dict[str, TopicObservation]:
        observations: dict[str, TopicObservation] = {}
        for contract in self._contracts:
            publisher_info = self.get_publishers_info_by_topic(contract.name)
            subscriber_info = self.get_subscriptions_info_by_topic(contract.name)
            rate = self._rates.sample(contract.name, now)
            observations[contract.name] = TopicObservation(
                publisher_count=len(publisher_info),
                subscriber_count=len(subscriber_info),
                publisher_types=self._endpoint_types(publisher_info),
                subscriber_types=self._endpoint_types(subscriber_info),
                sample_count=rate.sample_count,
                rate_hz=rate.rate_hz,
                age_sec=rate.age_sec,
            )
        return observations

    def _observe_tf(self) -> tuple[dict[str, bool], list[ContractViolation]]:
        timeout = Duration(
            seconds=max(0.0, float(self.get_parameter("tf_timeout_sec").value))
        )
        availability: dict[str, bool] = {}
        violations: list[ContractViolation] = []
        for requirement in self._tf_requirements:
            try:
                available = self._tf_buffer.can_transform(
                    requirement.target_frame,
                    requirement.source_frame,
                    Time(),
                    timeout=timeout,
                )
            except Exception:  # noqa: BLE001 - diagnostics must keep reporting
                available = False
            availability[requirement.label] = bool(available)
            if not available:
                violations.append(
                    ContractViolation(
                        "/tf",
                        "transform_missing",
                        f"{requirement.target_frame} <- "
                        f"{requirement.source_frame} unavailable",
                    )
                )
        return availability, violations

    @staticmethod
    def _observation_payload(observation: TopicObservation) -> dict[str, Any]:
        return {
            "publishers": observation.publisher_count,
            "subscribers": observation.subscriber_count,
            "publisher_types": sorted(observation.publisher_types),
            "subscriber_types": sorted(observation.subscriber_types),
            "samples": observation.sample_count,
            "rate_hz": (
                None
                if observation.rate_hz is None
                else round(observation.rate_hz, 3)
            ),
            "age_sec": (
                None
                if observation.age_sec is None
                else round(observation.age_sec, 3)
            ),
        }

    def _report(self) -> None:
        now = time.monotonic()
        elapsed = now - self._started_monotonic
        in_grace_period = elapsed < self._startup_grace_sec
        observations = self._observe_topics(now)
        violations = evaluate_contracts(
            self._contracts,
            observations,
            in_grace_period=in_grace_period,
        )
        tf_availability, tf_violations = self._observe_tf()
        if not in_grace_period:
            violations.extend(tf_violations)

        payload = {
            "ok": not violations and not in_grace_period,
            "in_grace_period": in_grace_period,
            "elapsed_sec": round(elapsed, 3),
            "simulator_state_owned": False,
            "cmd_vel_type": "geometry_msgs/msg/Twist",
            "topics": {
                name: self._observation_payload(observation)
                for name, observation in sorted(observations.items())
            },
            "tf": tf_availability,
            "violations": [
                {
                    "topic": violation.topic,
                    "code": violation.code,
                    "detail": violation.detail,
                }
                for violation in violations
            ],
        }
        status = String()
        status.data = json.dumps(payload, sort_keys=True)
        self._status_publisher.publish(status)

        signature = tuple(
            sorted(
                f"{violation.topic}:{violation.code}:{violation.detail}"
                for violation in violations
            )
        )
        if in_grace_period:
            log_signature = ("startup_grace",)
            if log_signature != self._last_log_signature:
                self.get_logger().info(
                    "Isaac contract startup grace active; collecting endpoints and rates"
                )
            self._last_log_signature = log_signature
            self._consecutive_failure_reports = 0
            return

        if not violations:
            if signature != self._last_log_signature:
                self.get_logger().info("Isaac ROS contract OK")
            self._consecutive_failure_reports = 0
        else:
            if signature != self._last_log_signature:
                summary = "; ".join(
                    f"{violation.topic} {violation.code}: {violation.detail}"
                    for violation in violations
                )
                self.get_logger().error("Isaac ROS contract failed: " + summary)
            self._consecutive_failure_reports += 1
            if (
                self._shutdown_on_failure
                and self._consecutive_failure_reports
                >= self._failure_reports_before_shutdown
            ):
                self.get_logger().fatal(
                    "Shutting down after repeated Isaac ROS contract failures"
                )
                rclpy.shutdown()
        self._last_log_signature = signature


def main(args=None) -> None:
    """Run the contract checker."""

    rclpy.init(args=args)
    node = IsaacContractChecker()
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
