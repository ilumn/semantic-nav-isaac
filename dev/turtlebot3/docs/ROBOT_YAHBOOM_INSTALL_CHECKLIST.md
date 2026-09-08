# Yahboom Jetson Robot Install Checklist

This checklist corresponds to `ROBOT_YAHBOOM_INSTALL_PLAN.md`.

## 0. Planning Gate

- [x] Confirm local screen sessions exist: `bot1`, `bot2`.
- [x] Confirm both sessions are already authenticated to `jetson@yahboom`.
- [x] Capture robot OS and Jetson Linux version.
- [x] Capture visible base/camera devices.
- [x] Capture current disk pressure.
- [x] Capture current ROS distribution.
- [x] Save detailed install plan.
- [x] Save detailed install checklist.
- [ ] Confirm source transfer route.
- [ ] Confirm whether this session may delete reviewed cache/build artifacts to
  free disk.
- [ ] Confirm robot is physically safe before any motion test.

## 1. Access And Transfer Prerequisites

- [ ] Decide transfer path: SSH/rsync through `neural`, Git remote pull, or
  bundle/tarball relay.
- [ ] If using SSH/rsync, authorize local key on `neural`.
- [ ] If using Git pull, push or otherwise expose the deployment branch/commit.
- [ ] If using Git pull, confirm the main repo branch includes vendored
  `dev/turtlebot3/external/semantic-nav-memory`.
- [ ] Confirm robot can reach required package sources or Git remotes.
- [ ] Create `~/semantic-nav` or backup existing `~/semantic-nav`.
- [ ] Confirm source checkout does not include build/install/log artifacts.

## 2. Disk Audit

- [x] Confirm root filesystem free space is only about 8 GB.
- [x] Run `df -h / /home`.
- [ ] Run `sudo du -xh --max-depth=1 / 2>/dev/null | sort -h`.
- [x] Run `du -xh --max-depth=1 ~ 2>/dev/null | sort -h`.
- [x] Run `docker system df`.
- [x] Inspect `~/.cache`.
- [x] Inspect `~/.ros/log`.
- [x] Inspect existing workspaces for `build/`, `install/`, and `log/`.
- [x] Inspect Docker images/containers before pruning.
- [x] Identify enough cleanup candidates for at least 15 GB free.
- [x] Preferably identify enough cleanup candidates for 25 GB free.

## 3. Disk Cleanup

- [ ] Run `sudo apt clean` if approved.
- [ ] Remove old ROS logs if approved.
- [ ] Remove old colcon build/install/log directories if approved.
- [ ] Prune unused Docker containers if approved.
- [ ] Prune unused Docker images if approved.
- [ ] Remove unused model/cache directories only after review.
- [ ] Recheck `df -h / /home`.
- [ ] Stop if free space remains below 15 GB.

## 4. Host ROS Dependency Install

- [ ] Run `sudo apt update`.
- [ ] Install build tools: `build-essential`, `cmake`, `git`, `rsync`, `curl`,
  `wget`, `unzip`.
- [ ] Install Python tools: `python3-pip`, `python3-venv`,
  `python3-colcon-common-extensions`, `python3-rosdep`, `python3-vcstool`.
- [ ] Install TurtleBot3 Humble packages.
- [ ] Install Nav2 Humble packages.
- [ ] Install `slam_toolbox`.
- [ ] Install message/runtime packages: `vision_msgs`, `cv_bridge`,
  `image_transport`, `camera_info_manager`, `v4l2_camera`, `tf2_ros`.
- [ ] Install robot state/URDF tools: `xacro`, `robot_state_publisher`,
  `joint_state_publisher`.
- [ ] Verify package prefixes with `ros2 pkg prefix`.
- [ ] Do not run broad `apt upgrade` during first install.

## 5. Python Runtime

- [ ] Create `~/semantic-nav/.venv`.
- [ ] Upgrade pip/setuptools/wheel in the venv.
- [ ] Install `ultralytics`.
- [ ] Install `opencv-python-headless`.
- [ ] Install `pyyaml`.
- [ ] Confirm CPU detector import works.
- [ ] Defer Jetson PyTorch/CUDA tuning until CPU-first validation passes.
- [ ] Defer `pycolmap` until live stack is stable.

## 6. Source Deployment

- [ ] Put the deployment branch/commit containing this cleanup on the robot.
- [ ] Confirm vendored `semantic-nav-memory` source is present on the robot.
- [ ] Confirm `~/semantic-nav/dev/turtlebot3/ros2_ws/src` exists.
- [ ] Confirm
  `~/semantic-nav/dev/turtlebot3/external/semantic-nav-memory/semantic_nav_memory/cli.py`
  exists.
- [ ] Confirm `tb3_semantic_map` package exists on robot.
- [ ] Confirm `tb3_semantic_refiner` package exists on robot.
- [ ] Confirm `tb3_semantic_map_msgs` package exists on robot.
- [ ] Confirm `real_semantic_nav.launch.py` exists on robot.
- [ ] Confirm `real_robot_preflight.py` exists on robot.
- [ ] Confirm no source tree contains stale generated build outputs.

## 7. Workspace Build

- [ ] Source `/opt/ros/humble/setup.bash`.
- [ ] Run `rosdep install --from-paths src --ignore-src -r -y`.
- [ ] Build message packages first if needed.
- [ ] Build Python packages.
- [ ] Build C++ frontier exploration package.
- [ ] Source `install/setup.bash`.
- [ ] Run package import smoke checks.
- [ ] Run available unit tests that do not require hardware motion.
- [ ] Recheck disk after build.

## 8. Hardware Topic Validation

- [ ] Start official TurtleBot3 bringup with robot physically safe.
- [ ] Confirm `/dev/ttyACM0` is usable by `jetson`.
- [ ] Confirm `/scan`.
- [ ] Confirm `/odom`.
- [ ] Confirm `/joint_states`.
- [ ] Confirm `odom -> base_link` TF.
- [ ] Identify which camera device is RGB: `/dev/video0` or `/dev/video1`.
- [ ] Start camera driver.
- [ ] Confirm `/camera/image_raw`.
- [ ] Confirm `/camera/camera_info`.
- [ ] Confirm camera frame name.
- [ ] Add/fix camera static transform if missing.
- [ ] Confirm `base_link -> camera frame` TF.

## 9. Real Stack Unarmed Launch

- [ ] Launch `real_semantic_nav.launch.py` with `detector_device:=cpu`.
- [ ] Launch with `refiner_worker_enabled:=false`.
- [ ] Launch with `refiner_auto_run:=false`.
- [ ] Confirm `motion_initially_armed:=false`.
- [ ] Confirm `exploration_initially_enabled:=false`.
- [ ] Confirm `/coordinator_node/status` reports unarmed.
- [ ] Confirm `/semantic_map/state` exists.
- [ ] Confirm `/semantic_map/markers` exists.
- [ ] Confirm no fake/bootstrap semantic objects appear.
- [ ] Run `ros2 run tb3_coordinator real_robot_preflight`.

## 10. Perception Validation

- [ ] Confirm detector node starts without crashing.
- [ ] Confirm YOLO weights are present in a runtime asset location.
- [ ] Confirm detections are from live camera frames.
- [ ] Confirm a person/object appears only when visible to the camera.
- [ ] Confirm semantic map entity count changes from live detections.
- [ ] Confirm stale detections decay or stop updating.
- [ ] Confirm semantic query can return a visible object.
- [ ] Confirm semantic query returns no fake hardcoded objects.

## 11. Controlled Motion

- [ ] Confirm operator approval before any motion.
- [ ] Confirm wheels lifted or bounded test area.
- [ ] Confirm `/cmd_vel` idle before arming.
- [ ] Arm with `/coordinator_node/set_motion_armed`.
- [ ] Disarm immediately and verify stop behavior.
- [ ] Run one short frontier exploration segment.
- [ ] Disarm and inspect map/TF.
- [ ] Run one semantic query navigation command.
- [ ] Disarm and inspect final state.

## 12. Optional COLMAP Refiner

- [ ] Confirm live navigation is stable before enabling refiner.
- [ ] Confirm at least 25 GB free before installing/refiner jobs.
- [ ] Install `ffmpeg`.
- [ ] Install `colmap`.
- [ ] Install `pycolmap`.
- [ ] Confirm `semantic-nav-memory` worker command runs.
- [ ] Enable `refiner_worker_enabled:=true`.
- [ ] Confirm frame bundles are created under `/tmp/tb3_semantic_refiner_jobs`.
- [ ] Confirm failed jobs do not break live navigation.
- [ ] Confirm successful jobs publish reference-only overlays.
- [ ] Confirm COLMAP pointcloud is denser than placeholder/debug points.

## 13. Documentation And Commit Hygiene

- [ ] Record exact package install commands that were run.
- [ ] Record exact source transfer method.
- [ ] Record robot-specific camera device and camera frame.
- [ ] Record robot-specific ROS domain and network assumptions.
- [ ] Update the checklist after each completed implementation step.
- [ ] Commit any local docs/config changes after validation.
