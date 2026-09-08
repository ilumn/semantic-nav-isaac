"""Loading and strict validation for the declarative scene manifest."""

from __future__ import annotations

import json
import hashlib
from pathlib import Path
from typing import Any, Mapping, Sequence


class ManifestError(ValueError):
    """Raised when the scene manifest is incomplete or internally inconsistent."""


SENSOR_DATA_QOS_PROFILE = {
    "history": "keepLast",
    "depth": 5,
    "reliability": "bestEffort",
    "durability": "volatile",
    "deadline": 0.0,
    "lifespan": 0.0,
    "liveliness": "systemDefault",
    "leaseDuration": 0.0,
}


def sensor_data_qos_json(manifest: Mapping[str, Any]) -> str:
    """Return the full Isaac ROS bridge sensor-data QoS JSON contract."""

    configured = manifest["ros2"]["sensor_data_qos"]
    ordered = {key: configured[key] for key in SENSOR_DATA_QOS_PROFILE}
    return json.dumps(ordered, separators=(",", ":"))


def manifest_digest(manifest: Mapping[str, Any]) -> str:
    """Return the canonical digest stored on managed stages."""

    serializable = {key: value for key, value in manifest.items() if not str(key).startswith("_")}
    payload = json.dumps(serializable, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def managed_stage_metadata_matches(
    manifest: Mapping[str, Any],
    *,
    schema_version: Any,
    target_version: Any,
    digest: Any,
) -> bool:
    """Pure predicate shared by stage loading and local tests."""

    return (
        schema_version == int(manifest["schema_version"])
        and target_version == str(manifest["target"]["isaac_sim"])
        and digest == manifest_digest(manifest)
    )


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ManifestError(f"{path} must be an object")
    return value


def _sequence(value: Any, path: str, *, length: int | None = None) -> Sequence[Any]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ManifestError(f"{path} must be an array")
    if length is not None and len(value) != length:
        raise ManifestError(f"{path} must contain exactly {length} values")
    return value


def _number(value: Any, path: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ManifestError(f"{path} must be numeric")
    result = float(value)
    if positive and result <= 0.0:
        raise ManifestError(f"{path} must be greater than zero")
    return result


def _vector(value: Any, path: str, length: int, *, positive: bool = False) -> tuple[float, ...]:
    seq = _sequence(value, path, length=length)
    return tuple(_number(item, f"{path}[{index}]", positive=positive) for index, item in enumerate(seq))


def _required(mapping: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise ManifestError(f"{path}.{key} is required")
    return mapping[key]


def _validate_pose(value: Any, path: str) -> None:
    pose = _mapping(value, path)
    _vector(_required(pose, "xyz", path), f"{path}.xyz", 3)
    _vector(_required(pose, "rpy", path), f"{path}.rpy", 3)


def _validate_box(value: Any, path: str) -> None:
    box = _mapping(value, path)
    _vector(_required(box, "center", path), f"{path}.center", 3)
    _vector(_required(box, "size", path), f"{path}.size", 3, positive=True)


def validate_manifest(data: Mapping[str, Any]) -> Mapping[str, Any]:
    """Validate *data* and return it unchanged.

    Validation is intentionally strict for values that change physics, frames, or ROS
    interfaces.  Additional keys are allowed so the schema can be extended without
    breaking older composition code.
    """

    root = _mapping(data, "manifest")
    if _required(root, "schema_version", "manifest") != 1:
        raise ManifestError("manifest.schema_version must be 1")

    target = _mapping(_required(root, "target", "manifest"), "manifest.target")
    if _required(target, "isaac_sim", "manifest.target") != "6.0.1":
        raise ManifestError("manifest.target.isaac_sim must be 6.0.1")
    if str(_required(target, "physics_engine", "manifest.target")).lower() != "physx":
        raise ManifestError("manifest.target.physics_engine must be physx")
    if str(_required(target, "ros_distro", "manifest.target")).lower() != "jazzy":
        raise ManifestError("manifest.target.ros_distro must be jazzy")

    stage = _mapping(_required(root, "stage", "manifest"), "manifest.stage")
    if _required(stage, "up_axis", "manifest.stage") != "Z":
        raise ManifestError("manifest.stage.up_axis must be Z")
    meters_per_unit = _number(
        _required(stage, "meters_per_unit", "manifest.stage"),
        "manifest.stage.meters_per_unit",
        positive=True,
    )
    if abs(meters_per_unit - 1.0) > 1e-12:
        raise ManifestError("manifest.stage.meters_per_unit must be 1.0")
    _number(_required(stage, "physics_hz", "manifest.stage"), "manifest.stage.physics_hz", positive=True)
    _number(_required(stage, "render_hz", "manifest.stage"), "manifest.stage.render_hz", positive=True)

    warehouse = _mapping(_required(stage, "warehouse", "manifest.stage"), "manifest.stage.warehouse")
    _vector(
        _required(warehouse, "interior_size_m", "manifest.stage.warehouse"),
        "manifest.stage.warehouse.interior_size_m",
        2,
        positive=True,
    )
    _validate_box(_required(warehouse, "floor", "manifest.stage.warehouse"), "manifest.stage.warehouse.floor")
    walls = _sequence(_required(warehouse, "walls", "manifest.stage.warehouse"), "manifest.stage.warehouse.walls")
    if len(walls) != 4:
        raise ManifestError("manifest.stage.warehouse.walls must contain four walls")
    wall_names: set[str] = set()
    for index, wall_value in enumerate(walls):
        wall_path = f"manifest.stage.warehouse.walls[{index}]"
        wall = _mapping(wall_value, wall_path)
        name = str(_required(wall, "name", wall_path))
        if name in wall_names:
            raise ManifestError(f"duplicate wall name: {name}")
        wall_names.add(name)
        _validate_box(wall, wall_path)

    robot = _mapping(_required(root, "robot", "manifest"), "manifest.robot")
    prim_path = str(_required(robot, "prim_path", "manifest.robot"))
    if not prim_path.startswith("/World/"):
        raise ManifestError("manifest.robot.prim_path must be below /World")
    _validate_pose(_required(robot, "spawn", "manifest.robot"), "manifest.robot.spawn")
    spawn_rpy = _vector(robot["spawn"]["rpy"], "manifest.robot.spawn.rpy", 3)
    if any(abs(value) > 1e-12 for value in spawn_rpy):
        raise ManifestError("manifest.robot.spawn.rpy must be zero for relative odometry compensation")
    _number(_required(robot, "wheel_radius_m", "manifest.robot"), "manifest.robot.wheel_radius_m", positive=True)
    _number(
        _required(robot, "wheel_separation_m", "manifest.robot"),
        "manifest.robot.wheel_separation_m",
        positive=True,
    )
    joints = _sequence(_required(robot, "wheel_joints", "manifest.robot"), "manifest.robot.wheel_joints", length=2)
    if len(set(map(str, joints))) != 2:
        raise ManifestError("manifest.robot.wheel_joints must name two distinct joints")
    if not str(_required(robot, "urdf_env", "manifest.robot")):
        raise ManifestError("manifest.robot.urdf_env cannot be empty")
    if not _sequence(_required(robot, "urdf_candidates", "manifest.robot"), "manifest.robot.urdf_candidates"):
        raise ManifestError("manifest.robot.urdf_candidates cannot be empty")
    for key in ("base_footprint", "base_link", "imu_link", "lidar_link", "camera_link"):
        if not str(_required(robot, key, "manifest.robot")):
            raise ManifestError(f"manifest.robot.{key} cannot be empty")
    base_fixed = _mapping(
        _required(robot, "base_footprint_to_base_link", "manifest.robot"),
        "manifest.robot.base_footprint_to_base_link",
    )
    _vector(
        _required(base_fixed, "xyz", "manifest.robot.base_footprint_to_base_link"),
        "manifest.robot.base_footprint_to_base_link.xyz",
        3,
    )
    fixed_rpy = _vector(
        _required(base_fixed, "rpy", "manifest.robot.base_footprint_to_base_link"),
        "manifest.robot.base_footprint_to_base_link.rpy",
        3,
    )
    if any(abs(value) > 1e-12 for value in fixed_rpy):
        raise ManifestError("base_footprint_to_base_link.rpy must be zero for odometry compensation")
    fixed_xyz = _vector(
        base_fixed["xyz"],
        "manifest.robot.base_footprint_to_base_link.xyz",
        3,
    )
    if abs(fixed_xyz[0]) > 1e-12 or abs(fixed_xyz[1]) > 1e-12:
        raise ManifestError(
            "base_footprint_to_base_link.xyz must be Z-only for the planar odometry adapter"
        )

    sensors = _mapping(_required(root, "sensors", "manifest"), "manifest.sensors")
    lidar = _mapping(_required(sensors, "lidar", "manifest.sensors"), "manifest.sensors.lidar")
    camera = _mapping(_required(sensors, "camera", "manifest.sensors"), "manifest.sensors.camera")
    imu = _mapping(_required(sensors, "imu", "manifest.sensors"), "manifest.sensors.imu")
    for sensor, sensor_path in (
        (lidar, "manifest.sensors.lidar"),
        (camera, "manifest.sensors.camera"),
        (imu, "manifest.sensors.imu"),
    ):
        _number(_required(sensor, "rate_hz", sensor_path), f"{sensor_path}.rate_hz", positive=True)
        if not str(_required(sensor, "frame_id", sensor_path)):
            raise ManifestError(f"{sensor_path}.frame_id cannot be empty")
    _number(
        _required(lidar, "min_range_m", "manifest.sensors.lidar"),
        "manifest.sensors.lidar.min_range_m",
        positive=True,
    )
    max_range = _number(
        _required(lidar, "max_range_m", "manifest.sensors.lidar"),
        "manifest.sensors.lidar.max_range_m",
        positive=True,
    )
    if max_range <= float(lidar["min_range_m"]):
        raise ManifestError("manifest.sensors.lidar.max_range_m must exceed min_range_m")
    _number(
        _required(lidar, "samples_per_scan", "manifest.sensors.lidar"),
        "manifest.sensors.lidar.samples_per_scan",
        positive=True,
    )
    _vector(
        _required(camera, "resolution", "manifest.sensors.camera"),
        "manifest.sensors.camera.resolution",
        2,
        positive=True,
    )
    _number(
        _required(camera, "horizontal_fov_rad", "manifest.sensors.camera"),
        "manifest.sensors.camera.horizontal_fov_rad",
        positive=True,
    )
    clip = _vector(
        _required(camera, "clip_m", "manifest.sensors.camera"),
        "manifest.sensors.camera.clip_m",
        2,
        positive=True,
    )
    if clip[1] <= clip[0]:
        raise ManifestError("manifest.sensors.camera.clip_m far value must exceed near value")
    for key in (
        "linear_acceleration_filter_size",
        "angular_velocity_filter_size",
        "orientation_filter_size",
    ):
        value = _number(_required(imu, key, "manifest.sensors.imu"), f"manifest.sensors.imu.{key}", positive=True)
        if not value.is_integer():
            raise ManifestError(f"manifest.sensors.imu.{key} must be an integer")
    if float(imu["rate_hz"]) != float(stage["physics_hz"]):
        raise ManifestError("manifest.sensors.imu.rate_hz must equal manifest.stage.physics_hz")
    if str(imu["frame_id"]) != str(robot["imu_link"]):
        raise ManifestError("manifest.sensors.imu.frame_id must match manifest.robot.imu_link")

    ros2 = _mapping(_required(root, "ros2", "manifest"), "manifest.ros2")
    sensor_qos = _mapping(
        _required(ros2, "sensor_data_qos", "manifest.ros2"),
        "manifest.ros2.sensor_data_qos",
    )
    if dict(sensor_qos) != SENSOR_DATA_QOS_PROFILE:
        raise ManifestError(
            "manifest.ros2.sensor_data_qos must be the Jazzy sensor-data profile "
            f"{SENSOR_DATA_QOS_PROFILE}"
        )
    topics = _mapping(_required(ros2, "topics", "manifest.ros2"), "manifest.ros2.topics")
    expected_topics = {
        "clock": "/clock",
        "cmd_vel": "/cmd_vel",
        "odom": "/odom",
        "joint_states": "/joint_states",
        "tf": "/tf",
        "scan": "/scan",
        "imu": "/imu",
        "camera_image": "/camera/image_raw",
        "camera_info": "/camera/camera_info",
    }
    for key, expected in expected_topics.items():
        if topics.get(key) != expected:
            raise ManifestError(f"manifest.ros2.topics.{key} must be {expected}")
    frames = _mapping(_required(ros2, "frames", "manifest.ros2"), "manifest.ros2.frames")
    if frames.get("odom") != "odom" or frames.get("base") != "base_footprint":
        raise ManifestError("manifest.ros2.frames must use odom and base_footprint")
    if ros2.get("cmd_vel_message") != "geometry_msgs/msg/Twist":
        raise ManifestError("manifest.ros2.cmd_vel_message must be geometry_msgs/msg/Twist")
    ownership = _mapping(_required(ros2, "tf_ownership", "manifest.ros2"), "manifest.ros2.tf_ownership")
    if list(ownership.get("isaac", [])) != ["odom->base_footprint"]:
        raise ManifestError("Isaac must own only odom->base_footprint")
    if "odom->base_link" in sum(
        (list(value) for value in ownership.values() if isinstance(value, Sequence) and not isinstance(value, str)),
        [],
    ):
        raise ManifestError("no component may publish odom->base_link")

    targets = _sequence(_required(root, "semantic_targets", "manifest"), "manifest.semantic_targets")
    target_ids: set[str] = set()
    for index, target_value in enumerate(targets):
        target_path = f"manifest.semantic_targets[{index}]"
        item = _mapping(target_value, target_path)
        target_id = str(_required(item, "id", target_path))
        if target_id in target_ids:
            raise ManifestError(f"duplicate semantic target id: {target_id}")
        target_ids.add(target_id)
        _validate_pose(_required(item, "pose", target_path), f"{target_path}.pose")
        _validate_pose(_required(item, "asset_pose", target_path), f"{target_path}.asset_pose")
        _vector(_required(item, "asset_scale", target_path), f"{target_path}.asset_scale", 3, positive=True)
        if not _sequence(_required(item, "dae_candidates", target_path), f"{target_path}.dae_candidates"):
            raise ManifestError(f"{target_path}.dae_candidates cannot be empty")
        fallback = _mapping(_required(item, "fallback", target_path), f"{target_path}.fallback")
        if fallback.get("shape") not in {"box", "capsule", "cylinder"}:
            raise ManifestError(f"{target_path}.fallback.shape is unsupported")

    if target_ids != {"table", "person", "stop_sign"}:
        raise ManifestError("semantic targets must be exactly table, person, and stop_sign")
    return data


def load_manifest(path: str | Path) -> dict[str, Any]:
    """Load and validate a JSON scene manifest from *path*."""

    manifest_path = Path(path).expanduser().resolve()
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ManifestError(f"scene manifest does not exist: {manifest_path}") from exc
    except json.JSONDecodeError as exc:
        raise ManifestError(f"invalid JSON in {manifest_path}: {exc}") from exc
    validate_manifest(data)
    data["_manifest_path"] = str(manifest_path)
    return data
