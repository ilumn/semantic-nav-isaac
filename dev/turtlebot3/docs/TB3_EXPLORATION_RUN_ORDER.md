# TurtleBot3 Autonomous Exploration — Run Order Checklist

> Isaac port note: the simulator-specific commands below are retained as
> historical context. Use `dev/isaac_sim/run_sim.sh` followed by
> `dev/isaac_sim/run_stack.sh`; see `FULL_STACK_SIM_RUNTIME.md`.

Step-by-step launch order and verification for: **TurtleBot3** → **slam_toolbox** → **Nav2** → **explore_lite**.

Use separate terminals (or tmux panes) for each long-running process. Source your workspace before any step:  
`source /opt/ros/humble/setup.bash` and `source install/setup.bash` (from `ros2_ws`).

---

## Prerequisites (once per session)

- [ ] `ROS_DISTRO=humble`
- [ ] Workspace built: `ros2_ws/install/setup.bash` exists; `source` it
- [ ] (Sim only) `export TURTLEBOT3_MODEL=burger` (or `waffle` / `waffle_pi`) in each terminal

---

## Step 1: TurtleBot3 (simulation or bringup)

**Purpose:** Robot model, `/odom`, `/scan`, TF `odom` → `base_link`, `/cmd_vel`.

### 1.1 Launch

**Simulation (Gazebo):**
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
# For the supported simulation runtime, use:
# ros2 launch tb3_coordinator full_semantic_nav.launch.py
```

**Real robot (bringup):**
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

### 1.2 Verify

| Check | Command | Expected |
|-------|---------|----------|
| /odom | `ros2 topic echo /odom --once` | nav_msgs/Odometry, header.frame_id=odom |
| /scan | `ros2 topic echo /scan --once` | sensor_msgs/LaserScan |
| /cmd_vel | `ros2 topic list \| grep cmd_vel` | /cmd_vel present |
| TF odom→base_link | `ros2 run tf2_ros tf2_echo odom base_link` | Transform (Ctrl+C to stop) |

### 1.3 Commands to run after Step 1

```bash
ros2 topic list | grep -E "odom|scan|cmd_vel"
ros2 run tf2_ros tf2_echo odom base_link
```

### 1.4 Common failures

| Symptom | Likely cause |
|---------|----------------|
| No /odom or /scan | Bringup/sim not fully started; wrong TURTLEBOT3_MODEL; USB/port (real robot). |
| "Transform failed" for odom→base_link | Wait a few seconds after launch; check robot driver is publishing TF. |
| No /cmd_vel | Bringup/sim not running or wrong namespace. |

---

## Step 2: slam_toolbox (SLAM)

**Purpose:** Build map; publish `/map` (OccupancyGrid) and TF `map` → `odom`.

### 2.1 Launch

```bash
ros2 launch slam_toolbox online_async_launch.py
```

Use a params file that matches your setup (e.g. `use_sim_time:=true` for Gazebo). If the default launch does not set it, pass or override:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

(Adjust to your distro’s launch API if different.)

### 2.2 Verify

| Check | Command | Expected |
|-------|---------|----------|
| /map | `ros2 topic echo /map --once` | nav_msgs/OccupancyGrid, header.frame_id=map |
| TF map→odom | `ros2 run tf2_ros tf2_echo map odom` | Transform available |
| TF map→base_link | `ros2 run tf2_ros tf2_echo map base_link` | Transform available |

### 2.3 Commands to run after Step 2

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
```

### 2.4 Common failures

| Symptom | Likely cause |
|---------|----------------|
| /map not published | slam_toolbox waiting for /scan and/or /odom; check Step 1. |
| "Transform failed" map→odom | use_sim_time mismatch (sim vs real); or SLAM not yet received enough data. |
| Map stays empty | Robot not moving; or /scan empty / wrong frame. |

---

## Step 3: Nav2 (navigation stack)

**Purpose:** Costmaps, planner, controller; provide `navigate_to_pose` action server.

### 3.1 Launch

**TurtleBot3 + Nav2 (typical):**
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

For real robot, use `use_sim_time:=false`. Use a Nav2 params file that matches your robot (e.g. from `turtlebot3_navigation2` if installed):

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/path/to/nav2_params.yaml
```

If you use TurtleBot3’s Nav2 launch:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

### 3.2 Verify

| Check | Command | Expected |
|-------|---------|----------|
| navigate_to_pose | `ros2 action list` | `/navigate_to_pose` (or namespaced) present |
| Global costmap | `ros2 topic echo /global_costmap/costmap --once` | OccupancyGrid (may take a few seconds) |
| TF | `ros2 run tf2_ros tf2_echo map base_link` | Still valid (Nav2 uses existing TF) |

### 3.3 Commands to run after Step 3

```bash
ros2 action list
ros2 topic list | grep costmap
ros2 run tf2_ros tf2_echo map base_link
```

### 3.4 Common failures

| Symptom | Likely cause |
|---------|----------------|
| navigate_to_pose action missing | Nav2 not fully up; wrong launch; or BT/costmap failure on startup. |
| No /global_costmap/costmap | Costmap server not started or still initializing; check /map and TF. |
| "Transform failed" in Nav2 logs | use_sim_time mismatch; or map→odom→base_link broken (re-check Step 1 & 2). |

---

## Step 4: explore_lite (m-explore-ros2)

**Purpose:** Compute frontiers from map/costmap and send goals to Nav2 via `navigate_to_pose`.

### 4.1 Launch

From your workspace (with explore_lite built and sourced):

```bash
ros2 launch explore_lite explore.launch.py use_sim_time:=true
```

To use your own params file (e.g. TurtleBot3 config):

```bash
ros2 launch explore_lite explore.launch.py use_sim_time:=true
# And in a custom launch, pass your config: parameters=[your_explore_lite_params.yaml, {"use_sim_time": True}]
```

Or run the node with a params file:

```bash
ros2 run explore_lite explore --ros-args --params-file /path/to/explore_lite_params.yaml -p use_sim_time:=true
```

### 4.2 Verify

| Check | Command | Expected |
|-------|---------|----------|
| explore/status | `ros2 topic echo /explore/status` | explore_lite_msgs/ExploreStatus |
| Frontiers (if visualize=true) | RViz: MarkerArray on explore/frontiers or similar | Markers in map frame |
| Robot moves | Watch in Gazebo or RViz | Goals sent; robot drives toward frontiers |

### 4.3 Commands to run after Step 4

```bash
ros2 topic echo /explore/status
ros2 topic list | grep explore
```
explore_lite_params.yaml
### 4.4 Common failures

| Symptom | Likely cause |
|---------|----------------|
| "Waiting for costmap" / no map | /map (or /global_costmap/costmap) not published; or wrong costmap_topic in params. |
| "Transform from base_link to map" failed | TF chain broken or use_sim_time mismatch; re-check Steps 1–3. |
| No frontiers / status never updates | Map all free or all unknown; min_frontier_size too large; or costmap topic not updating. |
| Goals sent but robot does not move | Nav2 not receiving or rejecting goals; check /cmd_vel and Nav2 logs. |

---explore_lite_params.yaml

## Quick reference: launch order

| Order | Component | Launch (example) |
|-------|-----------|------------------|
| 1 | TurtleBot3 | `ros2 launch turtlebot3_gazebo empty_world.launch.py` (or robot.launch.py) |
| 2 | slam_toolbox | `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true` |
| 3 | Nav2 | `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true` (or turtlebot3_navigation2) |
| 4 | explore_lite | `ros2 launch explore_lite explore.launch.py use_sim_time:=true` |

Use **use_sim_time:=false** for real robot in steps 2–4.

---

## End-to-end verification (all steps running)

```bash
# Topics
ros2 topic list | grep -E "map|odom|scan|cmd_vel|explore|costmap"

# TF
ros2 run tf2_ros tf2_echo map base_link

# Action
ros2 action list

# Exploration status
ros2 topic echo /explore/status
```

If all of the above succeed and the robot receives and executes goals, the baseline is ready.
