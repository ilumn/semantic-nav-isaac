# Semantic Navigation on NVIDIA Isaac Sim

This repository runs the existing ROS 2 semantic-navigation stack against
NVIDIA Isaac Sim instead of Gazebo. Isaac owns physics, the Waffle Pi robot,
RGB camera, 2D LiDAR, odometry, joint state, simulation time, and velocity
control. ROS 2 Jazzy continues to own SLAM Toolbox, Nav2, frontier exploration,
YOLO perception, semantic memory/query, and RViz.

The port targets **Isaac Sim 6.0.1**, **Ubuntu 24.04**, and **ROS 2 Jazzy**.
The `tb3_*` ROS package names are retained deliberately so the tested semantic
nodes and message APIs do not need a risky rename.

## What changed

- Gazebo and `ros_gz_bridge` are no longer part of the supported runtime.
- `dev/isaac_sim/` owns the Isaac scene, host preflight, launch, and shutdown.
- `isaac_semantic_nav` is the ROS-facing Isaac bringup and contract checker.
- Nav2 publishes an ordinary `geometry_msgs/Twist` on `/cmd_vel`, as required
  by Isaac Sim's ROS 2 Twist subscriber.
- The localizer accepts both `[0, 2π)` and `[-π, π]` LaserScan layouts.
- The original 4 x 6 metre room and semantic-target poses are reproduced by
  the Isaac scene source. Legacy SDF/DAE files remain only as conversion
  references.

## Runtime contract

| Direction | Interface | Owner |
| --- | --- | --- |
| Isaac to ROS | `/clock`, `/joint_states`, `/odom`, `/tf` | Isaac Sim |
| Isaac to ROS | `/scan` (`sensor_msgs/LaserScan`), `/imu` | Isaac Sim |
| Isaac to ROS | `/camera/image_raw`, `/camera/camera_info` | Isaac Sim |
| ROS to Isaac | `/cmd_vel` (`geometry_msgs/Twist`) | Nav2/operator |
| ROS only | `map -> odom`, `/map` | SLAM Toolbox |
| ROS only | semantic topics and `navigate_to_pose` | Existing stack |

Only one simulator may publish `/clock`. Isaac must not publish `map -> odom`;
that transform belongs to SLAM Toolbox.

## Quick start

Isaac Sim and model weights are intentionally not stored in Git. After
installing Isaac Sim 6.0.1, clone the repository and create the local runtime:

```bash
git clone <repository-url> semantic-nav-isaac
cd semantic-nav-isaac
cp dev/isaac_sim/.env.example dev/isaac_sim/.env
# Edit ISAAC_SIM_ROOT in dev/isaac_sim/.env.
dev/isaac_sim/fetch_models.sh --download
dev/isaac_sim/bootstrap_runtime.sh --allow-download
python3 dev/isaac_sim/scene/tools/convert_semantic_assets.py --execute
dev/isaac_sim/build_ros.sh --clean-cache
dev/isaac_sim/preflight.sh
dev/isaac_sim/run_sim.sh
```

In a second terminal:

```bash
cd semantic-nav-isaac
dev/isaac_sim/run_stack.sh
```

Validate the simulator boundary at any time:

```bash
source /opt/ros/jazzy/setup.bash
source dev/turtlebot3/ros2_ws/install/setup.bash
ros2 run isaac_semantic_nav contract_checker
```

Shut down only processes started by this port:

```bash
dev/isaac_sim/shutdown.sh
```

See [dev/isaac_sim/README.md](dev/isaac_sim/README.md) for host/runtime details
and [docs/ISAAC_SIM_PORT.md](docs/ISAAC_SIM_PORT.md) for the architecture and
acceptance checklist. Team workflow and pull-request expectations are in
[CONTRIBUTING.md](CONTRIBUTING.md).

## Repository layout

```text
dev/isaac_sim/                         Isaac scene and operator scripts
dev/turtlebot3/ros2_ws/src/
  isaac_semantic_nav/                  Isaac ROS bringup and contract checker
  tb3_coordinator/                     Shared stack orchestration
  tb3_detector/                        YOLO RGB detector
  tb3_localizer/                       Camera/LiDAR fusion
  tb3_frontier_exploration/            Frontier selection and goal assignment
  tb3_semantic_* and tb3_query/         Semantic state, memory, query, refinement
docs/ISAAC_SIM_PORT.md                 Port boundary and verification contract
```

The real-robot launch remains available; the simulator port does not change its
hardware safety defaults.

## Validation status

The complete visible stack was last validated on 2026-08-25 with Isaac Sim
6.0.1, ROS 2 Jazzy, and a single RTX 5090 renderer. The live contract reported
zero violations; measured wall rates included 8.21 Hz for `/scan`, 17.23 Hz for
`/camera/image_raw`, and 24.64 Hz for `/odom`. Portable source checks run in CI;
GPU/Isaac behavior must still be checked on a compatible workstation before a
release.

## License

Project-authored code and documentation are available under the MIT License;
see [LICENSE](LICENSE). Third-party models and assets retain their own terms;
see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
