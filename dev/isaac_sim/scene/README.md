# Isaac Sim semantic-navigation scene

This directory is the reproducible Isaac Sim side of the TurtleBot3 semantic-navigation port. It targets exactly NVIDIA Isaac Sim 6.0.1, PhysX, and ROS 2 Jazzy. The launcher authors the warehouse, imports the Waffle Pi, creates its sensors, and builds the ROS 2 Action Graphs at runtime; a generated USD is intentionally not checked in or claimed to exist before that runtime succeeds.

## Requirements

- NVIDIA Isaac Sim 6.0.1 on Linux, launched with its own `python.sh`
- ROS 2 Jazzy and `turtlebot3_description` (the default URDF candidate is under `/opt/ros/jazzy`)
- `xacro` at `/opt/ros/jazzy/bin/xacro`, on `PATH`, or supplied with `--xacro-executable`
- Either the Assimp CLI or a C++ compiler plus the installed Assimp development library for the bundled DAE converter
- An RTX-capable NVIDIA GPU

The launcher explicitly passes `multi_gpu=False`. NVIDIA documents a fatal CUDA error 700 failure for RTX LiDAR on some multi-GPU systems in Isaac Sim 6.0.1, so this scene always uses single-GPU rendering even on a multi-GPU host.

## Prepare semantic assets

The generated cache is gitignored. Build deterministic OBJ/MTL intermediates from the original Gazebo Collada models before the first launch:

```bash
cd semantic-nav-isaac/dev/isaac_sim/scene
python3 tools/convert_semantic_assets.py --execute
```

If no `assimp` executable exists, the command builds `tools/dae_to_obj.cpp` against the local Assimp library. Every `--execute` invocation rebuilds the OBJ/MTL files from the declared DAE sources, copies the model textures, restores the stop-sign diffuse/specular bindings that Gazebo supplied through a material script, and verifies every `mtllib`, `map_Kd`, `map_Ks`, normal, and bump dependency. This makes regeneration independent of source/output timestamps. The enabled table and person are hard preflight requirements: launch fails with the generator command if either validated intermediate is absent. They are never silently replaced with primitive geometry.

At Isaac runtime, Omniverse Asset Converter performs the second stage, OBJ/glTF to USD, and the launcher waits for each conversion to complete before checking and referencing its output. The USD cache filename contains a SHA-256-derived key over the intermediate, all referenced MTL files, and all referenced textures; changing texture bytes therefore cannot reuse a stale USD.

## Launch

The host wrapper configures the ROS bridge before `SimulationApp` is
constructed. From the repository root, use:

```bash
dev/isaac_sim/run_sim.sh
```

Cyclone DDS is also supported when every ROS process uses
`rmw_cyclonedds_cpp`. Do not mix a sourced installation with a different set of
internal libraries. Advanced users can invoke the scene with Isaac's Python
wrapper after configuring the same environment:

```bash
"${ISAAC_SIM_ROOT}/python.sh" dev/isaac_sim/scene/run_semantic_nav.py
```

No arguments opens the interactive Isaac Sim window and keeps the composed stage in memory. Common alternatives are:

```bash
dev/isaac_sim/run_sim.sh --headless
dev/isaac_sim/run_sim.sh --stage dev/isaac_sim/scene/generated/semantic_nav.usd
dev/isaac_sim/run_sim.sh --stage dev/isaac_sim/scene/generated/semantic_nav.usd -- --rebuild-stage
```

`--stage-path` opens an existing managed stage or composes and saves it when the path is absent. Existing stages must carry the expected schema version, Isaac version, and exact manifest SHA-256; a stale stage is rejected and must be rebuilt. `--compose-only` saves (if a path was supplied) and exits without playing. `--urdf-path`, `--xacro-executable`, and `--asset-cache` override discovery and generated-output locations.

The installed `turtlebot3_waffle_pi.urdf` is not plain URDF despite its suffix. The launcher detects live xacro constructs after ignoring XML comments, runs `xacro ... namespace:=` into the runtime cache, verifies no expressions remain, and keeps the original ROS package mapping so `package://turtlebot3_description/...` meshes resolve.

## Scene contract

- Z-up, one stage unit per meter, 200 Hz physics, 60 Hz rendering
- Warehouse free-space bounds: X `[-2, 2]` m and Y `[-3, 3]` m
- TurtleBot3 start: `(-1.2, -1.2, 0.02)` m, zero RPY
- Enabled semantic targets: table at `(0.7, 1.5, 0)` and person at `(0.6, -2.2, 0)`; stop sign remains declaratively disabled to match the source scenario
- Differential drive: 0.033 m wheel radius, 0.288 m wheel separation, velocity-controlled left/right wheel joints

The declarative source of truth is `config/scene_manifest.json`. The launcher fails on geometry, frame, sensor, version, importer, converter, or graph-authoring contract violations rather than continuing with a partial scene.

## ROS 2 contract

| Interface | ROS type | Intended rate/frame |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | playback/render tick |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | subscription |
| `/odom` | `nav_msgs/msg/Odometry` | playback/render tick; `odom` to `base_footprint` |
| `/joint_states` | `sensor_msgs/msg/JointState` | playback/render tick |
| `/tf` | `tf2_msgs/msg/TFMessage` | Isaac publishes only `odom -> base_footprint` |
| `/scan` | `sensor_msgs/msg/LaserScan` | configured 20 Hz; `base_scan`; sensor-data QoS |
| `/imu` | `sensor_msgs/msg/Imu` | 200 Hz physics step; `imu_link`; sensor-data QoS |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | configured 60 Hz; `camera_rgb_optical_frame`; sensor-data QoS |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | configured 60 Hz; `camera_rgb_optical_frame`; sensor-data QoS |

The frame tree has one owner per edge:

```text
map                         SLAM/localization
└── odom                    Isaac does not publish map -> odom
    └── base_footprint      Isaac odometry and raw TF
        └── base_link       external robot_state_publisher
            └── robot links external robot_state_publisher
```

The imported URDF's fixed `base_footprint -> base_link` transform is 0.010 m in Z. Isaac Compute Odometry must observe the rigid `base_link`, so the graph analytically shifts its relative pose to the `base_footprint` origin using `p_bf = p_bl - (R_rel*t - t)` and applies the corresponding fixed-origin correction to local linear velocity. This is deliberately a planar adapter: the manifest rejects any XY fixed offset or initial RPY, and differential-drive motion is constrained to yaw. The only angular component is therefore Z, which is invariant between the world and body frames; it is wired directly without unverified quaternion-to-matrix graph nodes. Isaac never publishes `odom -> base_link`.

Scan, image, camera-info, and IMU publishers receive the full ROS sensor-data QoS JSON profile: keep-last depth 5, best-effort reliability, and volatile durability. In RViz, select Best Effort for those displays. The IMU graph is authored in OmniGraph's on-demand pipeline stage so `On Physics Step` executes once per PhysX step rather than once per render/playback update.

For Nav2/SLAM, run the external nodes with `use_sim_time:=true`, feed `/scan`, and let SLAM Toolbox, AMCL, or the selected localization stack own `map -> odom`. Run `robot_state_publisher` from the same expanded Waffle Pi description so it owns the fixed and articulated link tree. This Action Graph consumes un-stamped `geometry_msgs/msg/Twist`; configure Nav2's stamped-command option off (`enable_stamped_cmd_vel: false`, where exposed) or retain the port's command adapter.

## Local verification and limitations

Portable source checks do not require Isaac Sim:

```bash
dev/isaac_sim/check_source.sh
```

The full visible stack was validated on 2026-08-25 with Isaac Sim 6.0.1 and ROS
2 Jazzy. USD composition, OmniGraph authoring, physics, GPU rendering, SLAM,
Nav2, semantic perception, and the ROS contract completed with zero violations.
Observed wall rates were 8.21 Hz for `/scan`, 17.23 Hz for
`/camera/image_raw`, and 24.64 Hz for `/odom`; configured sensor tick rates are
higher to sustain those wall rates on the validated host. The camera noise
value remains declarative only: no camera-noise model or USD metadata is
authored. Every release still requires a live run because CI does not provide
Isaac Sim, an RTX GPU, or a display server.

## Official references

- [Isaac Sim 6.0.1 workflows and standalone Python](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/introduction/workflows.html)
- [ROS 2 installation and standalone bridge environment](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_ros.html)
- [Isaac Sim 6.0.1 known issues](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/overview/known_issues.html)
- [ROS 2 reference architecture](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/ros2_reference_architecture.html)
- [URDF import](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/importer_exporter/import_urdf.html)
- [ROS 2 OmniGraph migration for Isaac Sim 6](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/migration_guides/isaac_sim_6_0/ros2_omnigraph_migration.html)
- [ROS 2 transform trees and odometry](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/ros2_tutorials/tutorial_ros2_tf.html)
- [ROS 2 Navigation with Nav2](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/ros2_tutorials/tutorial_ros2_navigation.html)
