# Sim2Real Deployment Plan

> Historical planning record. The Gazebo wrapper described below was removed by
> the Isaac Sim port. The current simulator entrypoint is
> `isaac_semantic_nav/isaac_semantic_nav.launch.py`; the real-robot entrypoint
> remains `tb3_coordinator/real_semantic_nav.launch.py`.

## Target

Prepare this stack for a real TurtleBot3 Waffle Pi-class robot with:

- ROS 2 Jazzy on Ubuntu 24.04
- Stock TurtleBot3 base bringup and LDS
- RGB camera publishing `/camera/image_raw` and `/camera/camera_info`
- Upgraded onboard compute capable of running Nav2, Locate Anything, and best-effort COLMAP refinement
- Explicit operator arming before autonomous motion

The real robot should run the full stack onboard: SLAM, Nav2, frontier exploration, detector, localizer, semantic memory, native semantic map, semantic query, navigation adapter, coordinator, and semantic refiner.

## Current State

The codebase has a working simulation-centered launch path and reusable ROS nodes. The individual perception and semantic launches already default to `use_sim_time:=false`, but the operator-facing full-stack launch still owns Gazebo, bridges, Nav2, exploration, perception, semantics, and RViz in one file.

The immediate sim2real gap is orchestration, not a complete rewrite of perception. The stack needs a shared ROS bringup that both simulation and real robot entrypoints can include, with simulation-specific Gazebo pieces kept outside the shared stack.

## Architecture

The target launch layout is:

```text
full_semantic_nav.launch.py        # simulation wrapper
  -> Gazebo Sim
  -> ros_gz bridges
  -> robot_state_publisher for sim model
  -> semantic_nav_stack.launch.py

real_semantic_nav.launch.py        # real robot wrapper
  -> assumes turtlebot3_bringup robot.launch.py is already running
  -> semantic_nav_stack.launch.py

semantic_nav_stack.launch.py       # shared ROS stack
  -> SLAM Toolbox
  -> Nav2
  -> frontier exploration
  -> detector / localizer / memory
  -> native semantic map
  -> query / nav adapter
  -> coordinator / semantic map memory
  -> semantic refiner
  -> optional RViz
```

Simulation and hardware should differ by launch arguments and profile defaults, not by maintaining two different navigation stacks.

## Hardware Preconditions

Before launching this repo's real stack, the robot must already provide:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/camera/image_raw`
- `/camera/camera_info`
- `base_link`, `odom`, camera frames, and a stable camera-to-base transform

The expected external command is:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
```

This repo should not replace official TurtleBot3 low-level bringup.

## Motion Safety

Real hardware must default to motion disabled. The stack should come up, validate topics and transforms, and report readiness before exploration or semantic navigation can command motion.

The target operator flow is:

```bash
ros2 launch tb3_coordinator real_semantic_nav.launch.py
ros2 service call /coordinator_node/set_motion_armed std_srvs/srv/SetBool "{data: true}"
```

Disarming must cancel active goals and prevent new autonomous goals.

## Runtime Profiles

Add separate runtime profiles for simulation and hardware:

- `waffle_pi_sim`: `use_sim_time:=true`, Gazebo topics, sim odometry defaults
- `waffle_pi_real`: `use_sim_time:=false`, official TurtleBot3 topics, real camera calibration and odometry defaults

The shared stack should accept explicit launch arguments for:

- `use_sim_time`
- `nav2_params_file`
- `map_topic`
- `costmap_topic`
- `odom_topic`
- `scan_topic`
- `image_topic`
- `camera_info_topic`
- `query_memory_mode`
- `query_memory_topic`
- `query_semantic_map_topic`
- `query_output_frame`
- `launch_semantic_map`
- `launch_semantic_refiner`
- `refiner_worker_enabled`
- `refiner_auto_run`
- `use_rviz`

## Perception And Semantics

The real robot should keep the live semantic map authoritative for navigation. The refiner is an onboard enrichment layer only:

- It may publish `/semantic_map/refined_state`.
- It may publish COLMAP point clouds and markers.
- It must not block live detector, localizer, Nav2, or query behavior.
- It must not write back into SLAM `/map`.
- It must degrade cleanly if dependencies or compute are unavailable.

Detector and localizer assumptions must be made explicit in profile/config:

- YOLO model path
- inference device
- camera image topic
- camera info topic
- camera HFOV fallback
- camera-to-base extrinsics
- lidar scan topic

## Validation Strategy

Validation must proceed in this order:

1. Launch-file validation with `--show-args`.
2. Build validation with `colcon build`.
3. Simulation validation through the simulation wrapper.
4. Real robot bench validation with wheels off the ground or otherwise motion-safe.
5. Real robot floor validation in a bounded space.

Acceptance for sim2real readiness:

- The simulation wrapper and real wrapper share `semantic_nav_stack.launch.py`.
- Real launch never starts Gazebo or ros_gz bridges.
- Real launch defaults to `use_sim_time:=false`.
- The robot stays still before explicit arming.
- Required topics and TF are checked before motion.
- Frontier exploration produces Nav2 goals after arming.
- Semantic query navigation can drive to a visible semantic target.
- Refiner can enrich or degrade without blocking navigation.
