# TurtleBot3 Sim2Real Runbook

This runbook is for the real TurtleBot3 Waffle Pi deployment path. It assumes the
simulation stack has already been validated with
`isaac_semantic_nav/isaac_semantic_nav.launch.py` against Isaac Sim.

## Target Robot

- TurtleBot3 Waffle Pi-class base
- ROS 2 Jazzy runtime on Ubuntu 24.04, either natively or in a container on a
  vendor-supported Jetson Orin host
- Official TurtleBot3 bringup for base, LDS, odometry, TF, and robot state
- RGB camera publishing `/camera/image_raw` and `/camera/camera_info`
- Onboard compute capable of running Nav2, Locate Anything, and best-effort refiner jobs

For Jetson Orin hardware installation, OS selection, power wiring, camera setup,
and SSH handoff, start with
`dev/turtlebot3/docs/JETSON_ORIN_TB3_WAFFLE_SETUP_GUIDE.md`.

## Onboard Workspace Setup

Build this repo on the robot:

```bash
cd ~/semantic-nav/dev/turtlebot3/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Required ROS-side capabilities:

- `turtlebot3_bringup`
- `turtlebot3_navigation2`
- `nav2_bringup`
- `slam_toolbox`
- `tf2_ros`
- `nav_msgs`
- `sensor_msgs`
- `vision_msgs`
- `std_srvs`
- Isaac Sim and its ROS bridge are development-machine dependencies only; they
  are not required by the real robot launch

Required perception/runtime assets:

- YOLO weights available to `tb3_detector` and `tb3_semantic_map`
- Camera driver calibrated enough to publish valid `CameraInfo`
- Static or URDF-provided transform from `base_link` to the camera optical frame

Optional enrichment dependencies:

- Vendored `semantic-nav-memory` worker source available at
  `dev/turtlebot3/external/semantic-nav-memory`
- COLMAP available to the refiner worker
- Sufficient writable space under `/tmp/tb3_semantic_refiner_jobs`

If optional enrichment dependencies are missing, the live semantic map must still
run. The refiner is not allowed to block navigation.

## Real Robot Launch Order

Terminal 1: base robot bringup.

```bash
export TURTLEBOT3_MODEL=waffle_pi
source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Terminal 2: semantic navigation stack.

```bash
cd ~/semantic-nav/dev/turtlebot3/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tb3_coordinator real_semantic_nav.launch.py
```

The real launch does not start Gazebo, `ros_gz_bridge`, or `ros_gz_image`.
It uses repo-owned real robot parameters:

- `tb3_coordinator/config/nav2/waffle_pi_real_nav2.yaml`
- `tb3_coordinator/config/slam/waffle_pi_real_slam.yaml`

## Safety Arm And Disarm

The real launch starts unarmed. In this state:

- Frontier exploration is disabled.
- The startup rotation warmup is disabled.
- User semantic navigation commands are ignored.
- Nav2 goals from the coordinator are blocked.

Arm autonomous motion only after readiness checks pass:

```bash
ros2 service call /coordinator_node/set_motion_armed std_srvs/srv/SetBool "{data: true}"
```

Disarm immediately if behavior looks wrong:

```bash
ros2 service call /coordinator_node/set_motion_armed std_srvs/srv/SetBool "{data: false}"
```

Disarming cancels coordinator-owned Nav2 goals, disables frontier exploration,
and causes the frontier goal assignment node to cancel its active frontier goal.

## Readiness Checks Before Arming

Run these checks before calling `set_motion_armed`:

```bash
ros2 run tb3_coordinator real_robot_preflight
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /camera/image_raw --once
ros2 topic echo /camera/camera_info --once
ros2 topic echo /tf --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 topic echo /coordinator_node/status --once
```

Expected result:

- `/scan`, `/odom`, camera, and TF topics are fresh.
- `real_robot_preflight` exits with `PASS`.
- `odom -> base_link` is available.
- The camera frame is connected to `base_link`.
- `/coordinator_node/status` reports `armed=False` before arming.

`real_semantic_nav.launch.py` also enforces these checks in the coordinator. If
readiness is missing or stale, the arm service returns failure and leaves
autonomous motion disabled.

## Bench Test Procedure

Use a bench setup with wheels off the ground or the robot otherwise physically
restrained.

1. Start official TurtleBot3 bringup.
2. Start `real_semantic_nav.launch.py`.
3. Confirm `/cmd_vel` is idle while unarmed.
4. Confirm `/coordinator_node/status` reports `armed=False`.
5. Confirm camera, scan, odom, and TF readiness checks pass.
6. Arm with `set_motion_armed`.
7. Confirm frontier exploration can issue Nav2 goals.
8. Send a visible-object command, for example `go to the person`, and confirm a semantic goal is produced.
9. Disarm and confirm active motion stops.

## Bounded Floor Test Procedure

Use a small bounded space with an operator beside the robot.

1. Start with the robot unarmed.
2. Verify the map, TF tree, and camera feed in RViz.
3. Arm the robot.
4. Let frontier exploration drive a short distance.
5. Disarm and verify the robot stops.
6. Re-arm and test one semantic query navigation command.
7. Watch `/semantic_map/state`, `/semantic_map/markers`, and `/coordinator_node/status`.
8. Stop the run if the robot localizes semantic objects behind walls, repeats unsafe goals, or loses TF.

## Simulation Parity

The simulation and real robot entrypoints share `semantic_nav_stack.launch.py`.
Real robot differences are launch arguments and hardware-provided topics, not a
separate navigation implementation.
