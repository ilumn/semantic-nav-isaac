# NVIDIA Isaac Sim port

## Target stack

This port targets Isaac Sim 6.0.1 on Ubuntu 24.04 with ROS 2 Jazzy and the
default PhysX backend. Isaac Sim is the appropriate NVIDIA component here:
Isaac Lab is a robot-learning framework, and Isaac ROS supplies accelerated ROS
perception packages rather than a general-purpose simulator.

The first port keeps the installed TurtleBot3 Waffle Pi model. This preserves
the robot geometry and the sim-to-real path while replacing Gazebo's physics,
sensor, and bridge implementation with Isaac Sim.

## Ownership boundary

```text
Isaac Sim 6.0.1
  PhysX articulation and differential drive
  RGB camera and 360 degree RTX LiDAR
  simulation clock, joint state, wheel odometry, odom -> base_footprint
  /cmd_vel Twist subscriber
             |
             | ROS 2 Jazzy / Fast DDS
             v
Shared ROS stack
  robot_state_publisher: base_link -> sensors and wheel links
  SLAM Toolbox: map -> odom and /map
  Nav2 + frontier exploration: goals and /cmd_vel
  Locate Anything + localizer + memory: live semantic observations
  semantic query + adapter: text target -> NavigateToPose
  RViz: operator visualization
```

No ground-truth object pose is injected into the runtime semantic map. Isaac
semantic labels may be used as a regression oracle, but the authoritative state
must still arise from rendered RGB observations and the simulated LiDAR.

## Stable simulator contract

| Interface | Type | Rate / rule |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | exactly one publisher |
| `/joint_states` | `sensor_msgs/msg/JointState` | wheel joints present |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` to `base_footprint` |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | one unambiguous tree |
| `/scan` | `sensor_msgs/msg/LaserScan` | configured 20 Hz, `base_scan` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | configured 60 Hz, 640 x 480 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | same camera/frame |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Isaac subscribes; unstamped |

Expected TF ownership:

```text
map                         SLAM Toolbox
└── odom                    SLAM Toolbox
    └── base_footprint         Isaac odometry
        └── base_link          robot_state_publisher
            ├── base_scan      robot_state_publisher
            ├── camera_link    robot_state_publisher
            │   └── camera_rgb_optical_frame
            ├── imu_link
            └── wheel links
```

`map -> odom` must never be published by Isaac. Isaac owns
`odom -> base_footprint`; robot_state_publisher owns the fixed
`base_footprint -> base_link` edge. Publishing `odom -> base_link` as well would
give `base_link` two parents and invalidate the TF tree.

## Scene parity

The source scene reconstructs the original semantic warehouse as native USD
primitives:

- interior floor: 4 by 6 metres;
- 2.5 metre perimeter walls at approximately `x = +/-2.1`, `y = +/-3.1`;
- Waffle Pi spawn at `(-1.2, -1.2, 0.02)`;
- table at `(0.7, 1.5, 0)`;
- person at `(0.6, -2.2, 0)`, facing yaw pi;
- optional stop sign target.

The old SDF worlds and Collada models are retained as visual/conversion source
material only. The Isaac runner is authoritative.

## Verification layers

Checks that do not require Isaac Sim:

1. compile every Python launch and scene source file;
2. parse every YAML/JSON manifest;
3. run pure scene-geometry, contract, and LaserScan tests;
4. build all ROS packages with ROS 2 Jazzy;
5. run ROS package tests and inspect failures.

Checks that require Isaac Sim 6.0.1:

1. preflight sees a direct NVIDIA OpenGL/Vulkan renderer and ROS Jazzy;
2. one `/clock` publisher and no Gazebo process exist;
3. camera and scan rates meet the contract with Sensor Data QoS;
4. `/odom` agrees with `odom -> base_footprint`;
5. a plain `/cmd_vel` drives forward and turns with correct wheel order;
6. SLAM Toolbox builds `/map` and Nav2 reaches a clicked goal;
7. Locate Anything detects the rendered person/table targets;
8. `/semantic_map/state` starts empty, fills from real observations, and a
   semantic command reaches `NavigateToPose`;
9. GUI and headless modes both shut down cleanly through the scoped lifecycle
   script.

Passing source/build checks proves the port is internally consistent. It does
not substitute for the final rendered-sensor and physics checks on an installed
Isaac runtime.
