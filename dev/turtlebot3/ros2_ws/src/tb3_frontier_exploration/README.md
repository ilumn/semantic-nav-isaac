# `tb3_frontier_exploration`

This package detects map frontiers, selects reachable candidates, and sends
goals through Nav2's `navigate_to_pose` action. It is simulator-neutral; Isaac
Sim only supplies `/clock`, `/scan`, `/odom`, TF, and the `/cmd_vel` sink.

## Supported runtime

Start the complete Isaac and ROS flow from the repository root:

```bash
dev/isaac_sim/run_sim.sh
dev/isaac_sim/run_stack.sh
```

The ROS launch wrapper passes `/odom` consistently. `/odometry/filtered` is no
longer the simulation default because no robot-localization filter owns that
topic in this stack.

## Nodes

- `frontier_detection_node` consumes `/map` and the global costmap and publishes
  candidate frontiers and markers.
- `goal_assignment_node` filters/selects candidates and sends Nav2 goals.
- `startup_map_warmup_node.py` gates exploration until the initial map/TF/sensor
  contract is ready.

## Source assets

`worlds/` and `models/` contain the previous SDF/Collada source material used to
reproduce scene geometry and appearance. They are not installed as ROS runtime
assets and there is no `ros_gz_bridge` dependency in this port. The supported
scene source lives under `dev/isaac_sim/scene`.
