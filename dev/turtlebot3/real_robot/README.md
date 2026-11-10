# semnav: robot runtime

Everything that runs on the robot, as systemd user services. `semnav` is the front door.

    semnav status                health of sensors, pose, detector, memory, motion, services
    semnav up | down | reset     start / stop / restart the stack (reset = fresh map and memory)
    semnav explore | arm         arm with frontier exploration on | off
    semnav go "go to the door"   arm, send the command, wait, disarm
    semnav look door             turn on the spot until it is in memory
    semnav find the trash can    understands sentences and synonyms ("the sofa", "somewhere to
                                 throw this away"); goes if it is in memory, otherwise explores
                                 until it is, then goes
    semnav understand "..."      which object find would look for, without moving
    semnav stop | disarm         stop motion
    semnav dryrun on | off       on: everything runs, the wheels get zero velocity
    semnav logs [unit] [-f]      journald logs
    semnav restart <unit>        safety | base | scan | camera | camflip | foxglove | detimg | control | stack
    semnav sh                    shell with the ROS environment set up

## Layout

| Service | What it does |
|---|---|
| `semnav-safety` | `semnav_safety.py`: the only path to the motors, /cmd_vel to /cmd_vel_safe |
| `semnav-base` | OpenCR, odometry, IMU, LDS-02 lidar (official bringup) |
| `semnav-scan` | /scan to fixed 360-ray /scan_slam, because slam_toolbox needs a constant ray count |
| `semnav-camera`, `semnav-camflip` | USB camera, rotated 180 degrees: /camera_raw/* to /camera/* |
| `semnav-foxglove` | Foxglove bridge on the Tailscale address, port 8765 |
| `semnav-detimg` | compressed detector image for remote viewers |
| `semnav-control` | `semnav_control.py`: commands, health, watchdog (UNIX socket, about 0.1 s per command) |
| `semnav-stack` | on demand: SLAM, Nav2, YOLOE semantic map, memory, query, coordinator |

`semnav.target` starts everything except the stack at boot. Motion always starts disabled.

## Motion safety

`semnav_safety.py` applies its rules at 20 Hz whatever the source of the command: enable state,
dry run, stale inputs, lidar corridor ahead and behind, per-direction turn clearance, and wheel
stall. Its limits: anything below lidar height is invisible to it, and a wheel that slips while
still turning does not count as a stall.

## Perception to memory

Frame -> YOLOE (one label per box) -> grounding (lidar range along the camera ray, using the scan
and pose at capture time; cut-off boxes and physically implausible widths are dropped) -> live
entities -> entities seen recently and repeatedly -> long-term landmarks -> query.

Objects below lidar height (a can on the floor) have no lidar return of their own, so they can be
placed at whatever stands behind them.

## Developing

Source is in `/mnt/data/semantic-nav-isaac/dev/turtlebot3/ros2_ws`; after changing a package:

    cd $SEMNAV_WS && colcon build --packages-select <pkg> --symlink-install

Real-robot settings: `tb3_coordinator/config/nav2/waffle_pi_real_nav2.yaml`, `slam/waffle_pi_real_slam.yaml`,
`tb3_coordinator/config/coordinator.yaml`, `tb3_semantic_map/config/semantic_map.yaml`.

`tools/record_frames.py` saves frames from a run for reviewing the vision model.
