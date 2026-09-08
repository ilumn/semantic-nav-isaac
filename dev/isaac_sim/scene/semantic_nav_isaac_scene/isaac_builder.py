"""Isaac Sim 6.0.1 scene authoring and ROS 2 Action Graph composition.

Import this module only after constructing ``isaacsim.SimulationApp``.  All API
names in this file are from the Isaac Sim 6 namespace.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import carb
import omni.graph.core as og
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.semantics as semantics_utils

from .conversion import find_existing_intermediate, intermediate_asset_fingerprint
from .geometry import horizontal_focal_length_mm, rpy_to_quaternion, warehouse_interior_bounds
from .manifest import managed_stage_metadata_matches, manifest_digest, sensor_data_qos_json
from .runtime_args import infer_ros_package_mapping, prepare_importable_urdf, scene_root_for_manifest


class SceneBuildError(RuntimeError):
    """Raised when Isaac cannot compose a complete, usable scene."""


@dataclass
class RobotPaths:
    root: str
    articulation_root: str
    base_footprint: str
    chassis: str
    imu_link: str
    lidar_link: str
    camera_link: str
    wheel_joints: tuple[str, str]


@dataclass
class BuildResult:
    stage: Usd.Stage
    robot: RobotPaths
    lidar_prim: str
    camera_prim: str
    imu_prim: str
    semantic_asset_modes: dict[str, str]
    warnings: list[str] = field(default_factory=list)


REQUIRED_EXTENSIONS = (
    "isaacsim.core.version",
    "isaacsim.asset.importer.urdf",
    "isaacsim.core.nodes",
    "omni.graph.nodes_core",
    "isaacsim.robot.wheeled_robots",
    "isaacsim.robot.wheeled_robots.nodes",
    "isaacsim.sensors.experimental.rtx",
    "isaacsim.sensors.experimental.physics",
    "isaacsim.sensors.physics.nodes",
    "isaacsim.ros2.core",
    "isaacsim.ros2.nodes",
    "isaacsim.ros2.bridge",
    "omni.kit.asset_converter",
)


def enable_required_extensions(simulation_app: Any) -> None:
    failures: list[str] = []
    for extension in REQUIRED_EXTENSIONS:
        if not app_utils.enable_extension(extension):
            failures.append(extension)
    simulation_app.update()
    if failures:
        raise SceneBuildError(f"Isaac Sim could not enable required extensions: {', '.join(failures)}")


def assert_supported_runtime(manifest: Mapping[str, Any]) -> str:
    """Fail unless the running app is exactly the declared Isaac/PhysX target."""

    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.version import get_version

    version = get_version()
    actual_version = ".".join(str(part) for part in version[2:5])
    expected_version = str(manifest["target"]["isaac_sim"])
    if actual_version != expected_version:
        raise SceneBuildError(
            f"this scene source targets Isaac Sim {expected_version}; running version is {actual_version}"
        )

    active_engine = SimulationManager.get_active_physics_engine()
    if active_engine != "physx":
        if not SimulationManager.switch_physics_engine("physx", verbose=True):
            raise SceneBuildError(
                f"PhysX is required, but active physics engine is {active_engine!r} and switching failed"
            )
        active_engine = SimulationManager.get_active_physics_engine()
    if active_engine != "physx":
        raise SceneBuildError(f"PhysX activation did not take effect; active engine is {active_engine!r}")
    return actual_version


def _annotate_managed_stage(stage: Usd.Stage, manifest: Mapping[str, Any]) -> None:
    world = stage.GetPrimAtPath("/World")
    world.CreateAttribute("semanticNav:schemaVersion", Sdf.ValueTypeNames.Int).Set(int(manifest["schema_version"]))
    world.CreateAttribute("semanticNav:targetIsaacSim", Sdf.ValueTypeNames.String).Set(
        str(manifest["target"]["isaac_sim"])
    )
    world.CreateAttribute("semanticNav:manifestSha256", Sdf.ValueTypeNames.String).Set(manifest_digest(manifest))


def validate_managed_stage(stage: Usd.Stage, manifest: Mapping[str, Any]) -> None:
    world = stage.GetPrimAtPath("/World")
    if not world:
        raise SceneBuildError("existing stage has no /World prim")
    schema = world.GetAttribute("semanticNav:schemaVersion")
    version = world.GetAttribute("semanticNav:targetIsaacSim")
    digest = world.GetAttribute("semanticNav:manifestSha256")
    if not schema or not version or not digest:
        raise SceneBuildError("existing --stage-path is not a managed semantic-nav Isaac stage")
    if not managed_stage_metadata_matches(
        manifest,
        schema_version=schema.Get(),
        target_version=version.Get(),
        digest=digest.Get(),
    ):
        raise SceneBuildError(
            "existing managed stage metadata does not match the current manifest; "
            "pass --rebuild-stage to recompose it"
        )


def open_managed_stage(simulation_app: Any, path: str | Path, manifest: Mapping[str, Any]) -> Usd.Stage:
    stage_path = Path(path).resolve()
    if not omni.usd.get_context().open_stage(str(stage_path)):
        raise SceneBuildError(f"Isaac Sim failed to open stage: {stage_path}")
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise SceneBuildError(f"USD context has no stage after opening {stage_path}")
    validate_managed_stage(stage, manifest)
    return stage


def save_stage(stage: Usd.Stage, path: str | Path) -> Path:
    output = Path(path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if not stage.GetRootLayer().Export(str(output)):
        raise SceneBuildError(f"USD root layer export failed: {output}")
    return output


def _gf_quaternion(wxyz: Sequence[float]) -> Gf.Quatd:
    w, x, y, z = (float(value) for value in wxyz)
    return Gf.Quatd(w, Gf.Vec3d(x, y, z))


def _set_local_transform(
    prim: Usd.Prim,
    *,
    xyz: Sequence[float] = (0.0, 0.0, 0.0),
    wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    scale: Sequence[float] | None = None,
) -> None:
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(float(value) for value in xyz)))
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(_gf_quaternion(wxyz))
    if scale is not None:
        xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(float(value) for value in scale)))


def _set_display_color(gprim: UsdGeom.Gprim, color: Sequence[float]) -> None:
    gprim.CreateDisplayColorAttr([Gf.Vec3f(*(float(value) for value in color))])


def _create_box(
    stage: Usd.Stage,
    path: str,
    center: Sequence[float],
    size: Sequence[float],
    color: Sequence[float],
    *,
    collision: bool = True,
) -> Usd.Prim:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    _set_local_transform(cube.GetPrim(), xyz=center, scale=size)
    _set_display_color(cube, color)
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


def _create_environment(stage: Usd.Stage, manifest: Mapping[str, Any]) -> None:
    stage_cfg = manifest["stage"]
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, float(stage_cfg["meters_per_unit"]))
    stage.SetTimeCodesPerSecond(float(stage_cfg["render_hz"]))

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, "/World/Environment")

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    physics_hz = int(round(float(stage_cfg["physics_hz"])))
    if not physx_scene.GetTimeStepsPerSecondAttr().Set(physics_hz):
        raise SceneBuildError(f"failed to set PhysX timestep rate to {physics_hz} Hz")

    warehouse = stage_cfg["warehouse"]
    bounds = warehouse_interior_bounds(warehouse["walls"])
    expected_width, expected_depth = (float(value) for value in warehouse["interior_size_m"])
    actual_width, actual_depth = bounds[1] - bounds[0], bounds[3] - bounds[2]
    if abs(actual_width - expected_width) > 1e-9 or abs(actual_depth - expected_depth) > 1e-9:
        raise SceneBuildError(
            f"wall geometry encloses {actual_width} x {actual_depth} m, not declared "
            f"{expected_width} x {expected_depth} m"
        )

    floor = warehouse["floor"]
    _create_box(
        stage,
        "/World/Environment/Floor",
        floor["center"],
        floor["size"],
        floor["color"],
    )
    UsdGeom.Scope.Define(stage, "/World/Environment/Walls")
    for wall in warehouse["walls"]:
        _create_box(
            stage,
            f"/World/Environment/Walls/{str(wall['name']).title()}",
            wall["center"],
            wall["size"],
            wall["color"],
        )

    light = UsdLux.DistantLight.Define(stage, "/World/Environment/Sun")
    light.CreateIntensityAttr(1000.0)
    light.CreateAngleAttr(0.53)
    _set_local_transform(light.GetPrim(), wxyz=rpy_to_quaternion((0.5, -0.6, -0.2)))
    dome = UsdLux.DomeLight.Define(stage, "/World/Environment/Ambient")
    dome.CreateIntensityAttr(350.0)
    dome.CreateColorAttr(Gf.Vec3f(0.7, 0.75, 0.8))


async def _convert_intermediate_to_usd(input_path: Path, output_path: Path) -> None:
    import omni.kit.asset_converter

    output_path.parent.mkdir(parents=True, exist_ok=True)
    context = omni.kit.asset_converter.AssetConverterContext()
    context.ignore_materials = False
    context.ignore_animations = True
    context.ignore_camera = True
    context.ignore_light = True
    context.single_mesh = False
    context.smooth_normals = True
    context.export_preview_surface = True

    def progress_callback(current_step: int, total_steps: int) -> None:
        carb.log_info(f"semantic asset conversion {input_path.name}: {current_step}/{total_steps}")

    converter = omni.kit.asset_converter.get_instance()
    task = converter.create_converter_task(str(input_path), str(output_path), progress_callback, context)
    if not await task.wait_until_finished():
        status = task.get_status()
        error = task.get_error_message()
        raise SceneBuildError(f"asset conversion failed for {input_path}: status={status}, error={error}")


def _add_collision_and_labels(root: Usd.Prim, label: str) -> None:
    semantics_utils.add_labels(root, labels=label)
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            semantics_utils.add_labels(prim, labels=label)


def _create_fallback_geometry(stage: Usd.Stage, root_path: str, fallback: Mapping[str, Any]) -> None:
    path = f"{root_path}/FallbackGeometry"
    shape = fallback["shape"]
    offset = fallback["center_offset"]
    color = fallback["color"]
    if shape == "box":
        _create_box(stage, path, offset, fallback["size"], color)
        return
    if shape == "capsule":
        gprim = UsdGeom.Capsule.Define(stage, path)
        gprim.CreateAxisAttr(UsdGeom.Tokens.z)
        gprim.CreateRadiusAttr(float(fallback["radius"]))
        gprim.CreateHeightAttr(float(fallback["height"]))
    elif shape == "cylinder":
        gprim = UsdGeom.Cylinder.Define(stage, path)
        gprim.CreateAxisAttr(UsdGeom.Tokens.z)
        gprim.CreateRadiusAttr(float(fallback["radius"]))
        gprim.CreateHeightAttr(float(fallback["height"]))
    else:
        raise SceneBuildError(f"unsupported fallback geometry: {shape}")
    _set_local_transform(gprim.GetPrim(), xyz=offset)
    _set_display_color(gprim, color)
    UsdPhysics.CollisionAPI.Apply(gprim.GetPrim())


def _create_semantic_targets(
    simulation_app: Any,
    stage: Usd.Stage,
    manifest: Mapping[str, Any],
    cache_dir: Path,
) -> tuple[dict[str, str], list[str]]:
    UsdGeom.Scope.Define(stage, "/World/SemanticTargets")
    modes: dict[str, str] = {}
    warnings: list[str] = []
    for target in manifest["semantic_targets"]:
        target_id = str(target["id"])
        if not bool(target["enabled"]):
            modes[target_id] = "disabled"
            continue
        root_path = f"/World/SemanticTargets/{target_id}"
        root = UsdGeom.Xform.Define(stage, root_path)
        _set_local_transform(
            root.GetPrim(),
            xyz=target["pose"]["xyz"],
            wxyz=rpy_to_quaternion(target["pose"]["rpy"]),
        )

        intermediate = find_existing_intermediate(manifest, target)
        if intermediate is None:
            raise SceneBuildError(
                f"{target_id}: no validated OBJ/glTF intermediate exists. Enabled semantic targets "
                "cannot use primitive fallbacks; run tools/convert_semantic_assets.py --execute first."
            )
        else:
            # A cache hit is valid only when the OBJ/glTF bytes and, for OBJ,
            # every MTL/texture byte match.  Timestamps alone miss changed
            # material dependencies and can silently preserve stale visuals.
            asset_fingerprint = intermediate_asset_fingerprint(intermediate, target)
            output_usd = (
                cache_dir
                / "semantic_assets"
                / "usd"
                / f"{target_id}-{asset_fingerprint[:16]}.usd"
            )
            if not output_usd.is_file():
                # SimulationApp.run_coroutine blocks by default; spell it out so
                # the output existence check cannot race the converter task.
                simulation_app.run_coroutine(
                    _convert_intermediate_to_usd(intermediate, output_usd),
                    run_until_complete=True,
                )
            if not output_usd.is_file():
                raise SceneBuildError(f"asset converter reported success but output does not exist: {output_usd}")
            asset_root = UsdGeom.Xform.Define(stage, f"{root_path}/Asset")
            _set_local_transform(
                asset_root.GetPrim(),
                xyz=target["asset_pose"]["xyz"],
                wxyz=rpy_to_quaternion(target["asset_pose"]["rpy"]),
                scale=target["asset_scale"],
            )
            asset_root.GetPrim().GetReferences().AddReference(str(output_usd))
            simulation_app.update()
            modes[target_id] = f"converted:{intermediate.suffix.lower()}:{asset_fingerprint[:16]}"
        _add_collision_and_labels(root.GetPrim(), str(target["label"]))
    return modes, warnings


def _find_named_descendant(root: Usd.Prim, name: str, *, kind: str) -> Usd.Prim:
    matches = [prim for prim in Usd.PrimRange(root) if prim.GetName() == name]
    if len(matches) != 1:
        paths = [str(prim.GetPath()) for prim in matches]
        raise SceneBuildError(
            f"expected exactly one imported {kind} named {name!r} below {root.GetPath()}, "
            f"found {len(matches)}: {paths}"
        )
    return matches[0]


def _find_articulation_root(root: Usd.Prim) -> Usd.Prim:
    matches = [prim for prim in Usd.PrimRange(root) if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(matches) != 1:
        paths = [str(prim.GetPath()) for prim in matches]
        raise SceneBuildError(
            f"expected one UsdPhysics.ArticulationRootAPI below {root.GetPath()}, found {len(matches)}: {paths}"
        )
    return matches[0]


def _configure_wheel_drives(wheel_joint_prims: Sequence[Usd.Prim]) -> None:
    for joint in wheel_joint_prims:
        if not joint.IsA(UsdPhysics.RevoluteJoint):
            raise SceneBuildError(f"wheel joint is not a revolute joint: {joint.GetPath()}")
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateTargetVelocityAttr().Set(0.0)
        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateDampingAttr().Set(1.0)
        drive.CreateMaxForceAttr().Set(20.0)


def _import_robot(
    simulation_app: Any,
    stage: Usd.Stage,
    manifest: Mapping[str, Any],
    source_urdf_path: Path,
    importable_urdf_path: Path,
    cache_dir: Path,
) -> RobotPaths:
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    robot_cfg = manifest["robot"]
    package_name = str(robot_cfg["ros_package_name"])
    # Package URLs in the expanded cache file still belong to the source ROS
    # package, so infer the importer mapping before leaving that package tree.
    package_mapping = infer_ros_package_mapping(source_urdf_path, package_name)
    output_dir = cache_dir / "robot"
    output_dir.mkdir(parents=True, exist_ok=True)
    import_config = URDFImporterConfig(
        urdf_path=str(importable_urdf_path),
        usd_path=str(output_dir),
        merge_fixed_joints=False,
        merge_mesh=False,
        collision_from_visuals=False,
        allow_self_collision=False,
        ros_package_paths=[package_mapping],
        robot_type="Wheeled",
        fix_base=False,
        joint_drive_type="force",
        joint_target_type="velocity",
        override_joint_stiffness=0.0,
        override_joint_damping=1.0,
        run_asset_transformer=True,
        run_multi_physics_conversion=True,
        debug_mode=False,
    )
    output_path = URDFImporter(import_config).import_urdf()
    if not output_path or not Path(output_path).is_file():
        raise SceneBuildError(
            f"URDF importer did not produce a robot USD for {importable_urdf_path}: {output_path!r}"
        )

    root_path = str(robot_cfg["prim_path"])
    robot_root = UsdGeom.Xform.Define(stage, root_path)
    _set_local_transform(
        robot_root.GetPrim(),
        xyz=robot_cfg["spawn"]["xyz"],
        wxyz=rpy_to_quaternion(robot_cfg["spawn"]["rpy"]),
    )
    robot_root.GetPrim().GetReferences().AddReference(str(Path(output_path).resolve()))
    simulation_app.update()

    root_prim = stage.GetPrimAtPath(root_path)
    articulation = _find_articulation_root(root_prim)
    base_footprint = _find_named_descendant(root_prim, str(robot_cfg["base_footprint"]), kind="link")
    chassis = _find_named_descendant(root_prim, str(robot_cfg["base_link"]), kind="link")
    imu_link = _find_named_descendant(root_prim, str(robot_cfg["imu_link"]), kind="link")
    lidar_link = _find_named_descendant(root_prim, str(robot_cfg["lidar_link"]), kind="link")
    camera_link = _find_named_descendant(root_prim, str(robot_cfg["camera_link"]), kind="link")
    wheel_prims = [
        _find_named_descendant(root_prim, str(name), kind="wheel joint") for name in robot_cfg["wheel_joints"]
    ]
    _configure_wheel_drives(wheel_prims)
    return RobotPaths(
        root=root_path,
        articulation_root=str(articulation.GetPath()),
        base_footprint=str(base_footprint.GetPath()),
        chassis=str(chassis.GetPath()),
        imu_link=str(imu_link.GetPath()),
        lidar_link=str(lidar_link.GetPath()),
        camera_link=str(camera_link.GetPath()),
        wheel_joints=tuple(str(prim.GetPath()) for prim in wheel_prims),
    )


def _set_required_sensor_attribute(prim: Usd.Prim, name: str, value: Any) -> None:
    attribute = prim.GetAttribute(name)
    if not attribute:
        raise SceneBuildError(f"sensor profile does not expose required attribute {name!r} on {prim.GetPath()}")
    if not attribute.Set(value):
        raise SceneBuildError(f"failed to author sensor attribute {name!r} on {prim.GetPath()}")


def _create_sensors(
    simulation_app: Any,
    stage: Usd.Stage,
    manifest: Mapping[str, Any],
    robot: RobotPaths,
) -> tuple[str, str, str]:
    from isaacsim.sensors.experimental.rtx import Lidar, RtxCamera
    from isaacsim.sensors.experimental.physics import IMU

    lidar_cfg = manifest["sensors"]["lidar"]
    lidar_path = f"{robot.lidar_link}/{lidar_cfg['name']}"
    lidar = Lidar.create(
        path=lidar_path,
        config=str(lidar_cfg["config"]),
        tick_rate=float(lidar_cfg["rate_hz"]),
        accumulate_outputs=True,
    )
    lidar_prim = stage.GetPrimAtPath(lidar.paths[0])
    _set_required_sensor_attribute(
        lidar_prim,
        "omni:sensor:Core:scanRateBaseHz",
        int(round(float(lidar_cfg["rate_hz"]))),
    )
    _set_required_sensor_attribute(
        lidar_prim,
        "omni:sensor:Core:reportRateBaseHz",
        int(round(float(lidar_cfg["rate_hz"]) * int(lidar_cfg["samples_per_scan"]))),
    )
    _set_required_sensor_attribute(lidar_prim, "omni:sensor:Core:nearRangeM", float(lidar_cfg["min_range_m"]))
    _set_required_sensor_attribute(lidar_prim, "omni:sensor:Core:farRangeM", float(lidar_cfg["max_range_m"]))
    _set_required_sensor_attribute(lidar_prim, "omni:sensor:Core:rangeAccuracyM", float(lidar_cfg["noise_stddev_m"]))
    _set_required_sensor_attribute(lidar_prim, "omni:sensor:Core:outputFrameOfReference", "SENSOR")

    camera_cfg = manifest["sensors"]["camera"]
    camera_path = f"{robot.camera_link}/{camera_cfg['name']}"
    camera = RtxCamera.create(path=camera_path, tick_rate=float(camera_cfg["rate_hz"]))
    camera.camera.set_apertures(
        float(camera_cfg["horizontal_aperture_mm"]),
        float(camera_cfg["horizontal_aperture_mm"])
        * float(camera_cfg["resolution"][1])
        / float(camera_cfg["resolution"][0]),
    )
    camera.camera.set_focal_lengths(
        horizontal_focal_length_mm(
            float(camera_cfg["horizontal_aperture_mm"]),
            float(camera_cfg["horizontal_fov_rad"]),
        )
    )
    camera.camera.set_clipping_ranges(float(camera_cfg["clip_m"][0]), float(camera_cfg["clip_m"][1]))
    camera_prim = stage.GetPrimAtPath(camera.paths[0])
    _set_local_transform(camera_prim, wxyz=camera_cfg["usd_camera_to_ros_optical_wxyz"])
    imu_cfg = manifest["sensors"]["imu"]
    imu_path = f"{robot.imu_link}/{imu_cfg['name']}"
    imu = IMU.create(
        path=imu_path,
        linear_acceleration_filter_size=int(imu_cfg["linear_acceleration_filter_size"]),
        angular_velocity_filter_size=int(imu_cfg["angular_velocity_filter_size"]),
        orientation_filter_size=int(imu_cfg["orientation_filter_size"]),
    )
    imu_prim = stage.GetPrimAtPath(imu.paths[0])
    if not imu_prim:
        raise SceneBuildError(f"physics IMU creation did not produce a prim at {imu_path}")
    simulation_app.update()
    return str(lidar_prim.GetPath()), str(camera_prim.GetPath()), str(imu_prim.GetPath())


def compose_scene(
    simulation_app: Any,
    manifest: Mapping[str, Any],
    *,
    urdf_path: str | Path,
    cache_dir: str | Path | None = None,
    xacro_executable: str | Path | None = None,
) -> BuildResult:
    """Compose the complete scene into a fresh in-memory USD stage."""

    context = omni.usd.get_context()
    context.new_stage()
    simulation_app.update()
    stage = context.get_stage()
    if stage is None:
        raise SceneBuildError("USD context did not create a new stage")

    resolved_cache = (
        Path(cache_dir).expanduser().resolve()
        if cache_dir is not None
        else scene_root_for_manifest(manifest) / "generated"
    )
    resolved_cache.mkdir(parents=True, exist_ok=True)
    source_urdf = Path(urdf_path).expanduser().resolve()
    try:
        importable_urdf = prepare_importable_urdf(
            source_urdf,
            resolved_cache,
            ros_distro=str(manifest["target"]["ros_distro"]),
            explicit_executable=xacro_executable,
        )
    except (FileNotFoundError, RuntimeError) as exc:
        raise SceneBuildError(str(exc)) from exc

    _create_environment(stage, manifest)
    semantic_modes, warnings = _create_semantic_targets(simulation_app, stage, manifest, resolved_cache)
    robot = _import_robot(
        simulation_app,
        stage,
        manifest,
        source_urdf,
        importable_urdf,
        resolved_cache,
    )
    lidar_prim, camera_prim, imu_prim = _create_sensors(simulation_app, stage, manifest, robot)
    _create_ros2_graphs(stage, manifest, robot, lidar_prim, camera_prim, imu_prim)
    _annotate_managed_stage(stage, manifest)
    simulation_app.update()
    return BuildResult(stage, robot, lidar_prim, camera_prim, imu_prim, semantic_modes, warnings)


# Graph construction lives below so all stage/sensor checks run before any ROS node is authored.


def _edit_graph(
    stage: Usd.Stage,
    graph_path: str,
    *,
    nodes: Sequence[tuple[str, str]],
    values: Sequence[tuple[str, Any]],
    connections: Sequence[tuple[str, str]],
    pipeline_stage: Any | None = None,
) -> None:
    """Author one replaceable execution graph and surface OGN schema errors."""

    if stage.GetPrimAtPath(graph_path):
        stage.RemovePrim(graph_path)
    graph_spec: dict[str, Any] = {"graph_path": graph_path}
    if pipeline_stage is None:
        graph_spec["evaluator_name"] = "execution"
    else:
        # NVIDIA's OnPhysicsStep pattern creates an on-demand graph without an
        # evaluator_name override; keep the exact supported graph spec.
        graph_spec["pipeline_stage"] = pipeline_stage
    try:
        og.Controller.edit(
            graph_spec,
            {
                og.Controller.Keys.CREATE_NODES: list(nodes),
                og.Controller.Keys.SET_VALUES: list(values),
                og.Controller.Keys.CONNECT: list(connections),
            },
        )
    except Exception as exc:
        raise SceneBuildError(f"failed to author ROS 2 Action Graph {graph_path}: {exc}") from exc


def _context_values(node: str, manifest: Mapping[str, Any]) -> list[tuple[str, Any]]:
    return [
        (
            f"{node}.inputs:useDomainIDEnvVar",
            bool(manifest["ros2"]["domain_id_from_environment"]),
        )
    ]


def _create_clock_graph(stage: Usd.Stage, manifest: Mapping[str, Any]) -> None:
    topics = manifest["ros2"]["topics"]
    _edit_graph(
        stage,
        "/World/ROS2/Clock",
        nodes=(
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("Publish", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ),
        values=(
            *_context_values("Context", manifest),
            ("Publish.inputs:topicName", str(topics["clock"])),
        ),
        connections=(
            ("Tick.outputs:tick", "Publish.inputs:execIn"),
            ("SimulationTime.outputs:simulationTime", "Publish.inputs:timeStamp"),
            ("Context.outputs:context", "Publish.inputs:context"),
        ),
    )


def _create_drive_graph(stage: Usd.Stage, manifest: Mapping[str, Any], robot: RobotPaths) -> None:
    robot_cfg = manifest["robot"]
    topics = manifest["ros2"]["topics"]
    max_wheel_speed = (
        float(robot_cfg["max_linear_speed_mps"]) / float(robot_cfg["wheel_radius_m"])
        + float(robot_cfg["max_angular_speed_rps"])
        * float(robot_cfg["wheel_separation_m"])
        / (2.0 * float(robot_cfg["wheel_radius_m"]))
    )
    _edit_graph(
        stage,
        "/World/ROS2/Drive",
        nodes=(
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("Subscribe", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("BreakLinear", "omni.graph.nodes.BreakVector3"),
            ("BreakAngular", "omni.graph.nodes.BreakVector3"),
            ("Differential", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("Articulation", "isaacsim.core.nodes.IsaacArticulationController"),
        ),
        values=(
            *_context_values("Context", manifest),
            ("Subscribe.inputs:topicName", str(topics["cmd_vel"])),
            ("Differential.inputs:wheelRadius", float(robot_cfg["wheel_radius_m"])),
            ("Differential.inputs:wheelDistance", float(robot_cfg["wheel_separation_m"])),
            ("Differential.inputs:maxLinearSpeed", float(robot_cfg["max_linear_speed_mps"])),
            ("Differential.inputs:maxAngularSpeed", float(robot_cfg["max_angular_speed_rps"])),
            ("Differential.inputs:maxWheelSpeed", max_wheel_speed),
            ("Articulation.inputs:targetPrim", [Sdf.Path(robot.articulation_root)]),
            ("Articulation.inputs:jointNames", [str(name) for name in robot_cfg["wheel_joints"]]),
        ),
        connections=(
            ("Tick.outputs:tick", "Subscribe.inputs:execIn"),
            ("Tick.outputs:deltaSeconds", "Differential.inputs:dt"),
            ("Context.outputs:context", "Subscribe.inputs:context"),
            ("Subscribe.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
            ("Subscribe.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
            ("Subscribe.outputs:execOut", "Differential.inputs:execIn"),
            ("BreakLinear.outputs:x", "Differential.inputs:linearVelocity"),
            ("BreakAngular.outputs:z", "Differential.inputs:angularVelocity"),
            ("Differential.outputs:velocityCommand", "Articulation.inputs:velocityCommand"),
            ("Tick.outputs:tick", "Articulation.inputs:execIn"),
        ),
    )


def _create_state_graph(stage: Usd.Stage, manifest: Mapping[str, Any], robot: RobotPaths) -> None:
    robot_cfg = manifest["robot"]
    topics = manifest["ros2"]["topics"]
    frames = manifest["ros2"]["frames"]
    _edit_graph(
        stage,
        "/World/ROS2/RobotState",
        nodes=(
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
            ("JointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("Odometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("RotateFootprintOffset", "omni.graph.nodes.RotateVector"),
            ("FootprintOffsetDelta", "omni.graph.nodes.Subtract"),
            ("SubtractFootprintOffset", "omni.graph.nodes.Subtract"),
            ("FootprintAngularOffset", "omni.graph.nodes.CrossProduct"),
            ("SubtractFootprintVelocity", "omni.graph.nodes.Subtract"),
            ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PublishTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ),
        values=(
            *_context_values("Context", manifest),
            ("ReadJointState.inputs:prim", [Sdf.Path(robot.articulation_root)]),
            ("JointState.inputs:topicName", str(topics["joint_states"])),
            # ComputeOdometry requires a rigid body/articulation root. The URDF's
            # empty base_footprint link is neither, so measure base_link and
            # analytically shift the relative pose and local twist to the fixed
            # base_footprint origin. For pose, subtract (R_rel * t - t), not
            # merely R_rel * t: ComputeOdometry's position starts at zero.
            ("Odometry.inputs:chassisPrim", [Sdf.Path(robot.chassis)]),
            (
                "RotateFootprintOffset.inputs:vector",
                {
                    "type": "vectord[3]",
                    "value": [
                        float(value)
                        for value in robot_cfg["base_footprint_to_base_link"]["xyz"]
                    ],
                },
            ),
            (
                "FootprintOffsetDelta.inputs:b",
                {
                    "type": "vectord[3]",
                    "value": [
                        float(value)
                        for value in robot_cfg["base_footprint_to_base_link"]["xyz"]
                    ],
                },
            ),
            (
                "FootprintAngularOffset.inputs:b",
                {
                    "type": "vectord[3]",
                    "value": [
                        float(value)
                        for value in robot_cfg["base_footprint_to_base_link"]["xyz"]
                    ],
                },
            ),
            ("PublishOdometry.inputs:topicName", str(topics["odom"])),
            ("PublishOdometry.inputs:odomFrameId", str(frames["odom"])),
            ("PublishOdometry.inputs:chassisFrameId", str(frames["base"])),
            # ComputeOdometry emits body-local linear velocity. Its PhysX
            # angular velocity can be wired directly for this strictly planar
            # contract: only Z is nonzero and Z is invariant under yaw. The
            # validator also requires the fixed link offset to be Z-only.
            ("PublishOdometry.inputs:publishRawVelocities", True),
            ("PublishTf.inputs:topicName", str(topics["tf"])),
            ("PublishTf.inputs:parentFrameId", str(frames["odom"])),
            ("PublishTf.inputs:childFrameId", str(frames["base"])),
        ),
        connections=(
            ("Tick.outputs:tick", "ReadJointState.inputs:execIn"),
            ("Tick.outputs:tick", "Odometry.inputs:execIn"),
            ("SimulationTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
            ("SimulationTime.outputs:simulationTime", "PublishTf.inputs:timeStamp"),
            ("Context.outputs:context", "JointState.inputs:context"),
            ("Context.outputs:context", "PublishOdometry.inputs:context"),
            ("Context.outputs:context", "PublishTf.inputs:context"),
            ("ReadJointState.outputs:execOut", "JointState.inputs:execIn"),
            ("ReadJointState.outputs:jointNames", "JointState.inputs:jointNames"),
            ("ReadJointState.outputs:jointPositions", "JointState.inputs:jointPositions"),
            ("ReadJointState.outputs:jointVelocities", "JointState.inputs:jointVelocities"),
            ("ReadJointState.outputs:jointEfforts", "JointState.inputs:jointEfforts"),
            ("ReadJointState.outputs:jointDofTypes", "JointState.inputs:jointDofTypes"),
            ("ReadJointState.outputs:stageMetersPerUnit", "JointState.inputs:stageMetersPerUnit"),
            ("ReadJointState.outputs:sensorTime", "JointState.inputs:sensorTime"),
            ("Odometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
            ("Odometry.outputs:execOut", "PublishTf.inputs:execIn"),
            ("Odometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
            ("Odometry.outputs:angularVelocity", "FootprintAngularOffset.inputs:a"),
            ("Odometry.outputs:linearVelocity", "SubtractFootprintVelocity.inputs:a"),
            ("FootprintAngularOffset.outputs:product", "SubtractFootprintVelocity.inputs:b"),
            ("SubtractFootprintVelocity.outputs:difference", "PublishOdometry.inputs:linearVelocity"),
            ("Odometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
            ("Odometry.outputs:orientation", "RotateFootprintOffset.inputs:rotation"),
            ("Odometry.outputs:position", "SubtractFootprintOffset.inputs:a"),
            ("RotateFootprintOffset.outputs:result", "FootprintOffsetDelta.inputs:a"),
            ("FootprintOffsetDelta.outputs:difference", "SubtractFootprintOffset.inputs:b"),
            ("SubtractFootprintOffset.outputs:difference", "PublishOdometry.inputs:position"),
            ("Odometry.outputs:orientation", "PublishTf.inputs:rotation"),
            ("SubtractFootprintOffset.outputs:difference", "PublishTf.inputs:translation"),
        ),
    )


def _create_lidar_graph(
    stage: Usd.Stage,
    manifest: Mapping[str, Any],
    lidar_prim: str,
) -> None:
    lidar_cfg = manifest["sensors"]["lidar"]
    topics = manifest["ros2"]["topics"]
    sensor_qos = sensor_data_qos_json(manifest)
    _edit_graph(
        stage,
        "/World/ROS2/Lidar",
        nodes=(
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("Publish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ),
        values=(
            *_context_values("Context", manifest),
            ("RenderProduct.inputs:cameraPrim", [Sdf.Path(lidar_prim)]),
            ("RenderProduct.inputs:width", 1),
            ("RenderProduct.inputs:height", 1),
            ("Publish.inputs:topicName", str(topics["scan"])),
            ("Publish.inputs:frameId", str(lidar_cfg["frame_id"])),
            ("Publish.inputs:type", "laser_scan"),
            ("Publish.inputs:qosProfile", sensor_qos),
            ("Publish.inputs:resetSimulationTimeOnStop", True),
        ),
        connections=(
            ("Tick.outputs:tick", "RenderProduct.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "Publish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "Publish.inputs:renderProductPath"),
            ("Context.outputs:context", "Publish.inputs:context"),
        ),
    )


def _create_camera_graph(
    stage: Usd.Stage,
    manifest: Mapping[str, Any],
    camera_prim: str,
) -> None:
    camera_cfg = manifest["sensors"]["camera"]
    topics = manifest["ros2"]["topics"]
    sensor_qos = sensor_data_qos_json(manifest)
    width, height = (int(value) for value in camera_cfg["resolution"])
    _edit_graph(
        stage,
        "/World/ROS2/Camera",
        nodes=(
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishImage", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("PublishInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ),
        values=(
            *_context_values("Context", manifest),
            ("RenderProduct.inputs:cameraPrim", [Sdf.Path(camera_prim)]),
            ("RenderProduct.inputs:width", width),
            ("RenderProduct.inputs:height", height),
            ("PublishImage.inputs:topicName", str(topics["camera_image"])),
            ("PublishImage.inputs:frameId", str(camera_cfg["frame_id"])),
            ("PublishImage.inputs:type", "rgb"),
            ("PublishImage.inputs:qosProfile", sensor_qos),
            ("PublishImage.inputs:resetSimulationTimeOnStop", True),
            ("PublishInfo.inputs:topicName", str(topics["camera_info"])),
            ("PublishInfo.inputs:frameId", str(camera_cfg["frame_id"])),
            ("PublishInfo.inputs:qosProfile", sensor_qos),
            ("PublishInfo.inputs:resetSimulationTimeOnStop", True),
        ),
        connections=(
            ("Tick.outputs:tick", "RenderProduct.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "PublishImage.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "PublishInfo.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "PublishImage.inputs:renderProductPath"),
            ("RenderProduct.outputs:renderProductPath", "PublishInfo.inputs:renderProductPath"),
            ("Context.outputs:context", "PublishImage.inputs:context"),
            ("Context.outputs:context", "PublishInfo.inputs:context"),
        ),
    )


def _create_imu_graph(
    stage: Usd.Stage,
    manifest: Mapping[str, Any],
    imu_prim: str,
) -> None:
    imu_cfg = manifest["sensors"]["imu"]
    topics = manifest["ros2"]["topics"]
    sensor_qos = sensor_data_qos_json(manifest)
    _edit_graph(
        stage,
        "/World/ROS2/Imu",
        nodes=(
            ("PhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("Read", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("Publish", "isaacsim.ros2.bridge.ROS2PublishImu"),
        ),
        values=(
            *_context_values("Context", manifest),
            ("Read.inputs:imuPrim", [Sdf.Path(imu_prim)]),
            ("Read.inputs:readGravity", bool(imu_cfg["read_gravity"])),
            ("Read.inputs:useLatestData", True),
            ("Publish.inputs:topicName", str(topics["imu"])),
            ("Publish.inputs:frameId", str(imu_cfg["frame_id"])),
            ("Publish.inputs:publishOrientation", True),
            ("Publish.inputs:qosProfile", sensor_qos),
        ),
        connections=(
            ("PhysicsStep.outputs:step", "Read.inputs:execIn"),
            ("Read.outputs:execOut", "Publish.inputs:execIn"),
            ("Read.outputs:angVel", "Publish.inputs:angularVelocity"),
            ("Read.outputs:linAcc", "Publish.inputs:linearAcceleration"),
            ("Read.outputs:orientation", "Publish.inputs:orientation"),
            ("Read.outputs:sensorTime", "Publish.inputs:timeStamp"),
            ("Context.outputs:context", "Publish.inputs:context"),
        ),
        pipeline_stage=og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    )


def _create_ros2_graphs(
    stage: Usd.Stage,
    manifest: Mapping[str, Any],
    robot: RobotPaths,
    lidar_prim: str,
    camera_prim: str,
    imu_prim: str,
) -> None:
    UsdGeom.Scope.Define(stage, "/World/ROS2")
    _create_clock_graph(stage, manifest)
    _create_drive_graph(stage, manifest, robot)
    _create_state_graph(stage, manifest, robot)
    _create_lidar_graph(stage, manifest, lidar_prim)
    _create_camera_graph(stage, manifest, camera_prim)
    _create_imu_graph(stage, manifest, imu_prim)
