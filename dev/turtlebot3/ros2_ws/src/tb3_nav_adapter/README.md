# `tb3_nav_adapter`

The navigation adapter converts a selected semantic target into a Nav2
`NavigateToPose` goal. It is simulator-neutral and never drives the robot
directly.

The adapter:

1. validates target frame and position;
2. computes a configurable standoff pose facing the object;
3. waits for the Nav2 action server;
4. sends the goal and reports acceptance/result status.

It consumes `/semantic_query_node/selected_target` and targets the standard
`navigate_to_pose` action. Motion arbitration and startup arming remain owned by
the coordinator.

For supported operation, start `dev/isaac_sim/run_stack.sh`. A semantic command
can then be sent with:

```bash
ros2 topic pub --once /semantic_query_node/command \
  std_msgs/msg/String "{data: 'go to the person'}"
```

The adapter sees only live ROS semantic state; Isaac scene coordinates are not
read or injected as navigation goals.
