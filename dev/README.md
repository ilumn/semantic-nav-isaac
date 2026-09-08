# Development runtimes

- `isaac_sim/` contains the supported NVIDIA Isaac Sim runtime, scene source,
  host checks, and process lifecycle scripts.
- `turtlebot3/ros2_ws/` contains the ROS 2 Jazzy workspace shared by Isaac Sim
  and the physical Waffle Pi-class robot.
- `turtlebot3/docs/` contains historical implementation and sim-to-real notes.

The `turtlebot3` path and `tb3_*` package names are API/layout compatibility
names. They do not imply that Gazebo is still the supported simulator.
