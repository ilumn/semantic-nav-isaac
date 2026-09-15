# Isaac Sim semantic navigation

This directory is the NVIDIA Isaac Sim 6.0.1 host wrapper for the TurtleBot3
semantic-navigation stack. Isaac owns physics, rendering, sensors, robot
control, and ROS bridge topics. The existing ROS 2 navigation, perception,
semantic memory, query, and frontier-exploration nodes run externally on ROS 2
Jazzy.

The default check, build, run, and shutdown paths do not download packages,
install software, start containers, or kill unrelated processes. Only the
bootstrap helper's explicit `--allow-download` mode permits network package
installation.

## Host requirements

- Native NVIDIA Isaac Sim **6.0.1**, with `python.sh` in its installation root.
- ROS 2 Jazzy at `/opt/ros/jazzy`.
- The `rmw_fastrtps_cpp` ROS package. Both Isaac and the external stack use Fast
  DDS and the same `ROS_DOMAIN_ID`.
- An NVIDIA RTX GPU and working driver. A visible run also needs a direct NVIDIA
  GL context on the configured `DISPLAY` (commonly `:0`).
- `colcon`, `flock`, and the workspace dependencies declared by the packages in
  `../turtlebot3/ros2_ws/src`.
- Either the Assimp CLI, or `g++` plus `libassimp-dev`, to regenerate the
  deterministic semantic OBJ/MTL intermediates from their source Collada files.
- A project-local `.venv` visible to Ubuntu's Python 3.12 with CUDA-enabled
  PyTorch, torchvision, Ultralytics, and OpenCV. `run_stack.sh` exports that
  venv's site-packages while retaining `/usr/bin/python3` for ROS Jazzy.

These helpers target a native Isaac installation. The scene explicitly uses one
GPU even on multi-GPU hosts.

## Configure and build

From this directory:

```bash
cp .env.example .env
# Edit ISAAC_SIM_ROOT in .env to point at the native 6.0.1 installation.

./bootstrap_runtime.sh --allow-download
./fetch_models.sh --download
python3 scene/tools/convert_semantic_assets.py --execute
./build_ros.sh
```

The conversion command is deterministic and safe to rerun. It fails if the
person/table meshes or any referenced MTL/texture cannot be rebuilt; enabled
semantic targets never fall back silently to primitives.

The pinned dependency set is recorded in `requirements-runtime.txt`:

```text
numpy==1.26.4
opencv-python==4.8.1.78
pydantic==2.13.2
pyyaml==6.0.3
torch==2.11.0
torchvision==0.26.0
transformers==4.57.1
tokenizers==0.22.0
accelerate==1.5.2
timm==1.0.22
peft==0.12.0
decord==0.6.0
lmdb==1.7.5
Pillow==11.1.0
huggingface-hub==0.36.0
ultralytics==8.4.38
ultralytics-thop==2.0.18
pycolmap==4.0.3
clip==1.0
ftfy==6.3.1
regex==2026.7.19
tqdm==4.70.0
wcwidth==0.8.2
```

Model checkpoints are not committed to Git. `fetch_models.sh --download`
retrieves the pinned LocateAnything-3B Hugging Face revision plus the exact
YOLOv8s-World-v2 and CLIP ViT-B/32 refinement assets. File assets must match
their SHA-256 digests. `fetch_models.sh --check` is network-free. Preflight
performs the same validation before launch.

The `clip==1.0` package is supplied by
`vendor/clip-1.0-py3-none-any.whl`, built from OpenAI CLIP commit
`d05afc436d78f1c48dc0dbf8e5980a9d471f35f6` under the MIT license. The
bootstrap helper passes this directory explicitly to `uv`, including in
offline mode.

The port does not install Python packages by default. Before launching the
detector, create a `.venv` at the sibling repository root with the GPU runtime
dependencies for this host, or point `ISAAC_SEMANTIC_VENV` at an equivalent
Python 3.12 venv.
Preflight checks that `/usr/bin/python3` can import the Locate Anything and
refiner dependencies, verifies the exact pinned versions,
imports Jazzy's `rclpy` and `cv_bridge` against NumPy 1.26.4, and confirms that
Torch sees at least one CUDA device. NumPy 2.x is intentionally excluded because
it is ABI-incompatible with the installed Jazzy `cv_bridge`. A portable venv
must be created locally; do not copy a venv from another checkout or machine.

The bootstrap helper is verification-only unless installation is explicitly
requested:

```bash
./bootstrap_runtime.sh --check
./bootstrap_runtime.sh --install-offline  # vendored wheels + uv cache; no network
./bootstrap_runtime.sh --allow-download   # explicitly permits package downloads
```

The model downloader has the same explicit-network convention:

```bash
./fetch_models.sh --check
./fetch_models.sh --download
```

Offline installation deliberately fails when a pinned wheel is absent instead
of falling back to the network. In particular, keep a local Python 3.12 wheel
for `pycolmap==4.0.3` if the uv cache does not contain one.

`ISAAC_SIM_PATH` and `ISAAC_SIM_ROOT` are both accepted. If neither is set, the
scripts inspect common native locations such as `/opt/isaac-sim`,
`/usr/local/isaac-sim`, `~/isaac-sim`, and legacy Omniverse package paths.
`ISAAC_PYTHON` can point directly at `python.sh` for an unusual layout.

`build_ros.sh` deliberately places `/usr/bin` first and passes
`-DPython3_EXECUTABLE=/usr/bin/python3`. This prevents CMake from accidentally
selecting a user-installed Python 3.11 instead of Jazzy's Ubuntu system Python.
It builds the shared workspace at `../turtlebot3/ros2_ws` with symlink install.
When a prior CMake configure cached the wrong interpreter, request a one-time
cache refresh:

```bash
./build_ros.sh --clean-cache
```

Run the non-destructive check before every first launch or after changing GPU,
display, ROS, or Isaac configuration:

```bash
./preflight.sh
# On a host without a display session:
./preflight.sh --headless
```

Preflight requires Isaac Sim 6.0.1 exactly. It also reports ROS Jazzy, the
workspace overlay, Fast DDS, every detected GPU/driver, GLX or headless state,
Vulkan probe availability, stale PID metadata, duplicate scene runners, and
competing Gazebo processes.

## Run

Terminal 1 starts the composed scene through Isaac's own Python launcher. A
pre-generated USD is not required:

```bash
./run_sim.sh
```

Headless mode uses the same scene composition path:

```bash
./run_sim.sh --headless
```

The default standalone entrypoint is `scene/run_semantic_nav.py`. Override it
with `--scene PATH` or `ISAAC_SCENE_SCRIPT`. To give the scene runner an
explicit generated-stage path:

```bash
./run_sim.sh --stage generated/semantic_nav.usd
```

Scene-specific arguments can follow `--`.

Terminal 2 launches the external ROS stack with simulation time enabled:

```bash
./run_stack.sh
```

Additional ROS launch arguments are passed through directly:

```bash
./run_stack.sh detector_device:=cuda:0 use_rviz:=true
```

The stack command defaults to:

```text
ros2 launch isaac_semantic_nav isaac_semantic_nav.launch.py use_sim_time:=true
```

Use `ISAAC_STACK_PACKAGE` or `ISAAC_STACK_LAUNCH_FILE` if a downstream package
renames that launch surface.

## Runtime contract

The simulator side is expected to publish `/clock`, `/joint_states`, `/odom`,
`/tf`, `/imu`, `/scan`, `/camera/image_raw`, and `/camera/camera_info`, and to
subscribe to `/cmd_vel`. The preserved frame contract is `odom` →
`base_footprint` → `base_link`, with `base_scan` and `camera_rgb_frame` sensor
frames.

Useful live checks after both terminals are running:

```bash
ros2 topic list -t
ros2 topic hz /clock
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

Only one `run_semantic_nav.py` scene runner is allowed. The launchers use locks
and PID records under `.runtime/` to reject duplicates.

## Shut down

Stop ROS consumers first and then Isaac:

```bash
./shutdown.sh
```

Narrow variants and a verification-only mode are available:

```bash
./shutdown.sh --stack-only
./shutdown.sh --sim-only
./shutdown.sh --dry-run
```

Shutdown never calls `pkill` and never searches for a name to kill. It reads
only this port's PID records, verifies each PID's Linux start time and command
marker against `/proc`, and sends `SIGTERM` to that dedicated process group or
exact PID. If a process exceeds the timeout, it is reported and left running;
no automatic `SIGKILL` is used.

If preflight reports a Gazebo process, shut down that separately using the
launcher that created it. This script intentionally refuses to manage the
original Gazebo project.
