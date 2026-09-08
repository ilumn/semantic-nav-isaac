# Isaac Semantic Navigation ROS Bringup

This ROS 2 Jazzy package connects the existing TurtleBot3 semantic-navigation
stack to NVIDIA Isaac Sim 6.0.1. It does not start or control Isaac Sim itself.
The simulator must own physics, `/clock`, sensor publication, odometry, and the
`/cmd_vel` subscription.

## Simulator contract

Isaac Sim must provide:

- `/clock` (`rosgraph_msgs/msg/Clock`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/odom` (`nav_msgs/msg/Odometry`)
- `/tf` and the `odom -> base_footprint` transform
- `/imu` (`sensor_msgs/msg/Imu`)
- `/scan` (`sensor_msgs/msg/LaserScan`, frame `base_scan`)
- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`)
- an unstamped `geometry_msgs/msg/Twist` subscription on `/cmd_vel`

The optional robot-state publisher supplies the Waffle Pi robot description,
fixed transforms, and joint transforms. In particular, Isaac must publish
`odom -> base_footprint`; the Waffle Pi URDF and robot-state publisher own
`base_footprint -> base_link`. SLAM owns `map -> odom`.

## Launch

Start the Isaac Sim stage first, then source the Jazzy workspace and run:

```bash
ros2 launch isaac_semantic_nav isaac_semantic_nav.launch.py
```

Useful bringup-only modes include:

```bash
# Inspect the simulator contract without starting Nav2 or semantic nodes.
ros2 launch isaac_semantic_nav isaac_semantic_nav.launch.py \
  launch_stack:=false use_rviz:=false

# Run without the asynchronous COLMAP refiner.
ros2 launch isaac_semantic_nav isaac_semantic_nav.launch.py \
  launch_semantic_refiner:=false
```

The contract checker is read-only with respect to the simulator. Its sole
publication is a latched JSON diagnostic on
`/isaac_semantic_nav/contract_status`. It uses a steady wall clock so it can
report a missing or paused `/clock` source.

## Velocity compatibility

`config/nav2_isaac_waffle_pi.yaml` is the conservative Waffle Pi Nav2 profile
with `enable_stamped_cmd_vel: false` for the controller, behavior server,
collision monitor, and velocity smoother. This ensures the final `/cmd_vel`
type remains `geometry_msgs/msg/Twist`, matching Isaac Sim's
`ROS2SubscribeTwist` graph node.
