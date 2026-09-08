# Yahboom Jetson Robot Install Plan

This plan is for installing the semantic navigation stack onto the real robot
reachable through the existing local screen sessions `bot1` and `bot2`.

No package installs, file deletion, motion launch, or robot arming should happen
until this plan and the matching checklist are saved.

## Confirmed Robot Facts

Observed through `bot1` / `bot2` on 2026-04-28:

- Hostname: `yahboom`
- Robot user: `jetson`
- OS: Ubuntu 22.04.5 LTS
- Kernel: `5.15.148-tegra` on `aarch64`
- Jetson Linux: R36.4.3
- ROS install: `/opt/ros/humble`
- Login banner: `ROS: humble`, `DOMAIN_ID: 99`
- Sourced ROS environment: `ROS_DISTRO=humble`, `ROS_DOMAIN_ID=99`
- Storage: `/dev/nvme0n1p1`, 157 GB total, 142 GB used, about 8 GB free
- Available tools: Docker 27.5.0, Git 2.34.1, Python 3.10.12, pip 24.3.1,
  `colcon`
- Devices visible: `/dev/ttyACM0`, `/dev/video0`, `/dev/video1`
- User groups include: `sudo`, `video`, `docker`, `gpio`, `i2c`, `render`
- ROS packages observed in the quick check: `cv_bridge`, `image_transport`
- ROS packages not observed in the quick check: TurtleBot3, Nav2, slam_toolbox

## Read-Only Disk Audit Results

Additional read-only audit after saving this plan found:

- `docker system df` reports 36.3 GB of Docker images, all reclaimable because
  there are no active containers.
- Docker images present:
  - `isaac_ros_dev-aarch64:latest` at 27.3 GB
  - `nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble...` at 26.7 GB
  - `ghcr.io/open-webui/open-webui:main` at 3.73 GB
  - `yahboomtechnology/ros-melodic:usb_cam` at 4.65 GB
  - `heartexlabs/label-studio:latest` at 662 MB
- `~/workspaces` is 15 GB, almost entirely
  `~/workspaces/isaac_ros-dev/isaac_ros_assets`.
- `~/.cache` is about 9.6 GB in the detailed pass:
  - `~/.cache/huggingface` is 5.5 GB
  - `~/.cache/uv` is 3.9 GB
  - `~/.cache/pip` is 310 MB
- Existing build/install/log directories found:
  - `~/yahboom_ws/build`
  - `~/yahboom_ws/install`
  - `~/yahboom_ws/log`
  - `~/jetcam/build`
  - `~/jetson-gpio/build`
  - `~/audio/build`
  - `~/install`
  - `~/.ros/log`
- Required ROS Humble apt packages are available from
  `http://packages.ros.org/ros2/ubuntu jammy/main arm64`.

Recommended cleanup order:

1. Remove only inactive Docker images if the user confirms they are not needed.
2. Remove Isaac ROS assets if the user confirms they are not needed.
3. Clean `~/.cache/huggingface` and `~/.cache/uv` only if those cached models or
   uv packages are not needed.
4. Clean old colcon build/install/log directories only after confirming no
   current Yahboom workspace depends on them.

## Important Constraint

The repo's latest sim2real docs target a ROS 2 Jazzy runtime, but this robot is
currently a JetPack 6 / Ubuntu 22.04 / ROS Humble system. Installing a native
Jazzy apt stack on this host is not the right first move. A Jazzy container is
possible, but the robot currently has only about 8 GB free, which is too tight
for a full container, ROS install, models, build output, logs, and refiner jobs.

The practical first deployment path is:

1. Preserve the robot's vendor-supported Ubuntu 22.04 / JetPack 6 host.
2. Build and run this repo as a ROS Humble overlay first.
3. Keep the real robot launch unarmed and refiner worker disabled initially.
4. Validate base bringup, camera topics, TF, SLAM/Nav2, semantic map, and query
   pieces incrementally.
5. Add GPU YOLO and COLMAP/refiner only after the live safety path works.
6. Revisit a Jazzy container only after there is enough disk and a proven reason
   to move off Humble.

This is intentionally robot-native and sim2real-friendly: it uses the robot's
actual ROS distribution, kernel, device paths, camera devices, OpenCR serial
path, and network domain instead of forcing a desktop-like environment.

## What Is Needed

### Access

- Keep `bot1` as the main robot terminal.
- Keep `bot2` as the spare terminal.
- Use `screen -x` / `screen -S <name> -X stuff` only; do not detach the user's
  terminals.
- For reliable file transfer, choose one of these before deployment:
  - Preferred: authorize this machine's SSH key on `neural` so `rsync`/`scp`
    can use `illumination@neural` as a jump host.
  - Acceptable: push the current main repo commit and the local
    `semantic-nav-memory` commit to Git remotes the robot can clone.
  - Fallback: create git bundles or tarballs and transfer them through `neural`.

### Disk Space

Target free space before install:

- Minimum for native Humble overlay without heavy models/refiner: 15 GB free.
- Recommended for YOLO model caches and build artifacts: 25 GB free.
- Recommended before any Jazzy container attempt: 40 GB free.

The robot currently has about 8 GB free, so the first implementation step must
be a disk audit. Do not delete large directories until they are listed and
reviewed.

### System Packages

Expected apt packages for the native Humble path:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git rsync curl wget unzip \
  python3-pip python3-venv python3-colcon-common-extensions \
  python3-rosdep python3-vcstool \
  ros-humble-turtlebot3 ros-humble-turtlebot3-bringup \
  ros-humble-turtlebot3-msgs ros-humble-turtlebot3-navigation2 \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-tf2-ros \
  ros-humble-vision-msgs ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-image-transport-plugins \
  ros-humble-camera-info-manager ros-humble-v4l2-camera \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-rviz2
```

Notes:

- `rviz2` is useful only if the robot has a GUI or X forwarding; otherwise RViz
  can run on a workstation.
- The exact camera driver may change after checking whether `/dev/video0` or
  `/dev/video1` is the usable RGB stream.
- Avoid broad `apt upgrade` during first deployment. The login banner reports
  hundreds of pending updates; upgrading the Jetson before robot bringup would
  add risk and time. Security/OS updates should be a separate maintenance task.

### Python Packages

Use a workspace-local virtual environment for non-ROS Python dependencies:

```bash
cd ~/semantic-nav
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install ultralytics pyyaml opencv-python-headless
```

COLMAP/refiner dependencies are optional for the first run:

```bash
python -m pip install pycolmap
sudo apt install -y ffmpeg colmap
```

Do not install PyTorch blindly through generic pip wheels on Jetson. Use the
JetPack-compatible PyTorch path only after CPU-first detection works and there
is enough free disk.

### Source Code

Required source on robot:

- Main repo commit containing the sim2real stack and vendored worker source.
- Vendored `semantic-nav-memory` source under
  `dev/turtlebot3/external/semantic-nav-memory`.

The vendored worker source is a tracked snapshot of:

```text
https://github.com/ilumn/semantic-nav-memory.git
c5ad423425a0ba2e8a2007d8368eaa4825949435
```

The old root-level `semantic-nav-memory/` checkout is ignored and is not needed
on the robot unless intentionally testing a local worker override.

### Runtime Assets

- YOLO weights should be stored under a model cache or explicit robot asset
  directory, not committed source.
- Camera calibration must provide `/camera/camera_info`.
- Camera TF must connect `base_link` to the camera frame.
- Real stack must start with:
  - `motion_initially_armed:=false`
  - `exploration_initially_enabled:=false`
  - `launch_startup_warmup:=false`
  - `refiner_worker_enabled:=false` for first validation

## Deployment Phases

### Phase 0 - Freeze Safety State

Goal: make sure install work cannot accidentally move the robot.

Actions:

1. Keep wheels off the ground or robot physically bounded if anything is
   launched.
2. Do not call `/coordinator_node/set_motion_armed`.
3. Do not run frontier exploration until preflight passes.
4. Use `bot1` for commands and `bot2` for observation or recovery.

### Phase 1 - Disk And Baseline Audit

Goal: understand what is consuming disk and what robot packages already exist.

Read-only commands:

```bash
df -h / /home
sudo du -xh --max-depth=1 / 2>/dev/null | sort -h
du -xh --max-depth=1 ~ 2>/dev/null | sort -h
docker system df
ls -lah ~
ls -lah ~/.cache ~/.ros 2>/dev/null || true
apt-cache policy ros-humble-turtlebot3-bringup ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-slam-toolbox
```

Cleanup candidates to review before deleting:

- Docker images/containers/volumes not needed by the robot
- Old ROS logs under `~/.ros/log`
- Old colcon `build/`, `install/`, and `log/` trees
- Large model caches under `~/.cache`, `~/.ultralytics`, Ollama, or local demos
- Apt package cache

### Phase 2 - Free Disk Space

Goal: reach at least 15 GB free before apt installs, preferably 25 GB.

Safe cleanup likely to be allowed:

```bash
sudo apt clean
rm -rf ~/.ros/log/*
```

Cleanup requiring review:

```bash
docker system prune
docker image prune -a
rm -rf <old workspace>/build <old workspace>/install <old workspace>/log
rm -rf <large unused cache path>
```

### Phase 3 - Install Host Dependencies

Goal: install ROS Humble packages needed for the native overlay.

Actions:

1. Run `sudo apt update`.
2. Install the apt package list above.
3. Run `rosdep update` if needed.
4. Confirm packages:

```bash
. /opt/ros/humble/setup.bash
ros2 pkg prefix turtlebot3_bringup
ros2 pkg prefix nav2_bringup
ros2 pkg prefix slam_toolbox
ros2 pkg prefix vision_msgs
```

### Phase 4 - Transfer Source

Goal: put the exact source commits on the robot.

Preferred if Git remotes have the commits:

```bash
cd ~
git clone <main-repo-url> semantic-nav
cd semantic-nav
git checkout <deployment-branch-or-commit>
test -f dev/turtlebot3/external/semantic-nav-memory/semantic_nav_memory/cli.py
```

Preferred if direct file transfer is available:

```bash
rsync -av --delete \
  --exclude '.git/' \
  --exclude 'dev/turtlebot3/ros2_ws/build/' \
  --exclude 'dev/turtlebot3/ros2_ws/install/' \
  --exclude 'dev/turtlebot3/ros2_ws/log/' \
  ./ illumination@neural:/tmp/semantic-nav/
```

Then copy from `neural` to the robot over the LAN or use a tarball.

Do not create a second `semantic-nav-memory` clone on the robot for normal
deployment. The refiner prefers the vendored worker source.

### Phase 5 - Build Workspace

Goal: build only the real robot overlay first.

Commands:

```bash
cd ~/semantic-nav/dev/turtlebot3/ros2_ws
. /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
. install/setup.bash
```

If the full build is too large or a package fails, build in layers:

```bash
colcon build --symlink-install --packages-select tb3_semantic_map_msgs
colcon build --symlink-install --packages-select tb3_detector tb3_semantic_map
colcon build --symlink-install --packages-select tb3_query tb3_nav_adapter
colcon build --symlink-install --packages-select tb3_coordinator
colcon build --symlink-install --packages-select tb3_frontier_exploration
```

### Phase 6 - Camera And Base Validation

Goal: verify hardware topics before semantic stack launch.

Base:

```bash
export TURTLEBOT3_MODEL=waffle_pi
. /opt/ros/humble/setup.bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Camera:

```bash
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

Validation:

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_link
```

The camera frame name may need adjustment after seeing the actual camera driver
output.

### Phase 7 - Launch Real Stack Unarmed

Goal: start the stack without autonomous motion.

```bash
cd ~/semantic-nav/dev/turtlebot3/ros2_ws
. /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch tb3_coordinator real_semantic_nav.launch.py \
  detector_device:=cpu \
  refiner_worker_enabled:=false \
  refiner_auto_run:=false \
  motion_initially_armed:=false \
  exploration_initially_enabled:=false
```

Run preflight:

```bash
ros2 run tb3_coordinator real_robot_preflight
ros2 topic echo /coordinator_node/status --once
ros2 topic echo /semantic_map/state --once
```

### Phase 8 - Perception Validation

Goal: verify live detections and semantic map updates without motion.

Actions:

1. Point the robot camera at a known object/person.
2. Confirm detector output topics exist.
3. Confirm `/semantic_map/state` updates from real perception.
4. Confirm markers do not contain fake/bootstrap detections.
5. Confirm query returns a real entity only after perception sees it.

### Phase 9 - Controlled Motion

Goal: allow motion only after all readiness checks pass.

Actions:

1. Bench test with wheels lifted.
2. Confirm `/cmd_vel` is idle before arming.
3. Arm with `set_motion_armed`.
4. Immediately disarm and confirm motion stops.
5. Run a short bounded-floor frontier test.
6. Run one semantic query navigation test.

### Phase 10 - Optional Refiner/COLMAP

Goal: add enrichment only after live navigation is stable.

Actions:

1. Install `ffmpeg`, `colmap`, and `pycolmap`.
2. Confirm enough disk for `/tmp/tb3_semantic_refiner_jobs`.
3. Launch with `refiner_worker_enabled:=true`.
4. Confirm failures degrade gracefully.
5. Confirm pointcloud overlays are reference-only and do not drive navigation.

## Stop Conditions

Stop and ask before continuing if:

- Free disk cannot be raised above 15 GB.
- Apt wants to remove core NVIDIA, ROS, or TurtleBot packages.
- A command requests a reboot.
- Base bringup produces unexpected motor motion.
- `/scan`, `/odom`, or TF are missing.
- Camera topics cannot provide valid `CameraInfo`.
- Build failures require source changes.
- Any step requires deleting unknown user data.

## Expected First Install Session Outcome

The first install session should aim for:

- Source present on robot.
- ROS Humble overlay built.
- Base bringup validated.
- Camera topics validated.
- Real semantic stack launched unarmed.
- Preflight run and documented.
- No autonomous motion unless explicitly approved after preflight.

GPU inference, COLMAP density, and autonomous exploration are second-session
goals unless the first session completes cleanly with enough disk and time.
