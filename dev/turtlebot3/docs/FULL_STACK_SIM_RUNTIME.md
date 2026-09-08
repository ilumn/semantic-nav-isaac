# Full-stack Isaac Sim runtime

This sibling supports one operator-facing simulation flow: Isaac Sim 6.0.1
provides the robot and rendered sensors, while ROS 2 Jazzy runs the existing
navigation and semantic stack.

## Start

Build once:

```bash
cd semantic-nav-isaac
dev/isaac_sim/build_ros.sh --clean-cache
```

Start the simulator from the repository root:

```bash
export ISAAC_SIM_PATH=/absolute/path/to/isaac-sim
dev/isaac_sim/preflight.sh
dev/isaac_sim/run_sim.sh
```

Start the ROS stack in a second terminal:

```bash
dev/isaac_sim/run_stack.sh
```

Run the contract checker in a third terminal when diagnosing the boundary:

```bash
source /opt/ros/jazzy/setup.bash
source dev/turtlebot3/ros2_ws/install/setup.bash
ros2 run isaac_semantic_nav contract_checker
```

## Runtime scope

- Isaac Sim PhysX robot and warehouse scene
- ROS 2 bridge clock, drive, odometry, TF, camera, and 2D LiDAR
- SLAM Toolbox and Nav2
- frontier exploration
- RGB YOLO detector and camera/LiDAR localizer
- persistent semantic map, query, and navigation goal adapter
- optional semantic refiner
- RViz

## Supported rules

- Exactly one Isaac process owns `/clock`.
- `/cmd_vel` is `geometry_msgs/msg/Twist`, not `TwistStamped`.
- Isaac publishes `odom -> base_footprint`; robot_state_publisher publishes
  `base_footprint -> base_link`; SLAM Toolbox publishes `map -> odom`.
- Semantic state starts empty and is populated only by rendered observations.
- Use `dev/isaac_sim/shutdown.sh` to stop the processes launched by this port.

Partial scene generation, fake detections, ground-truth semantic injection, and
stale refiner replays are not supported validation paths. A behavior counts as
validated only when it is reproducible through the full Isaac plus ROS flow.

See `docs/ISAAC_SIM_PORT.md` for the complete topic/TF contract and acceptance
checklist.
