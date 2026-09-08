# Shared TurtleBot3 ROS 2 stack

This directory contains the ROS 2 Jazzy workspace shared by NVIDIA Isaac Sim
and a real TurtleBot3 Waffle Pi-class robot. Simulation ownership lives in
`../isaac_sim`; the packages here consume a stable ROS topic and TF contract.

## Build

```bash
cd semantic-nav-isaac
dev/isaac_sim/build_ros.sh --clean-cache
source dev/turtlebot3/ros2_ws/install/setup.bash
```

## Isaac runtime

From the repository root, start Isaac and the ROS stack in separate terminals:

```bash
dev/isaac_sim/run_sim.sh
dev/isaac_sim/run_stack.sh
```

The ROS launch entrypoint is:

```bash
ros2 launch isaac_semantic_nav isaac_semantic_nav.launch.py
```

It uses simulation time, the repo-owned Isaac/Nav2 parameters, the Waffle Pi
robot description, and the generic interfaces `/scan`, `/odom`,
`/camera/image_raw`, `/camera/camera_info`, and `/cmd_vel`.

## Real robot

The real-robot path remains separate and conservative:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch tb3_coordinator real_semantic_nav.launch.py
```

See [../../docs/ISAAC_SIM_PORT.md](../../docs/ISAAC_SIM_PORT.md) for the port
boundary and `docs/SIM2REAL_REAL_ROBOT_RUNBOOK.md` for hardware operation.
