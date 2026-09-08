# Phase 1 Baseline: TurtleBot3 Autonomous Exploration

> Historical baseline: simulator references here describe the pre-port Gazebo
> implementation. Current operation is documented in `FULL_STACK_SIM_RUNTIME.md`.

## 1. Goal

Phase 1 established a **validated baseline** for autonomous frontier exploration in simulation: the robot builds a map with SLAM, and explore_lite sends Nav2 goals to frontiers without manual goal commands. The stack was verified in Gazebo with slam_toolbox, Nav2 in SLAM mode, and m-explore-ros2 (explore_lite).

---

## 2. Final Working Architecture

Single, consistent pipeline (no mixing of SLAM and localization in the same run):

```
Gazebo  →  slam_toolbox  →  Nav2 (slam:=true)  →  explore_lite
(robot)     (map + TF)       (costmaps + BT)      (frontiers → goals)
```

- **Gazebo:** Robot, `/odom`, `/scan`, TF `odom` → `base_link`.
- **slam_toolbox:** `/map`, TF `map` → `odom`.
- **Nav2 (slam:=true):** Uses `/map` and TF; provides costmaps and `navigate_to_pose` action.
- **explore_lite:** Reads `/map` (or global costmap), picks frontiers, sends goals via `navigate_to_pose`.

---

## 3. Terminal-by-Terminal Run Instructions

Use **four terminals**. In each, set the model and source the environment once:

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source /path/to/ros2_ws/install/setup.bash   # e.g. dev/turtlebot3/ros2_ws
```

**Terminal 1 — Gazebo (simulation):**
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Terminal 2 — slam_toolbox:**
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**Terminal 3 — Nav2 (SLAM mode; critical: do not use localization/AMCL here):**
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true slam:=true
```
(If your launch uses a different flag for “no map_server/AMCL, use live SLAM map”, set that instead; the point is Nav2 must use the live `/map` from slam_toolbox, not a static map + AMCL.)

**Terminal 4 — explore_lite:**
```bash
ros2 launch explore_lite explore.launch.py use_sim_time:=true
```

Drive the robot briefly in simulation so slam_toolbox has some map; then explore_lite will start sending goals and the robot will explore on its own.

---

## 4. Minimal Command List

```bash
# Environment (every terminal)
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source /path/to/ros2_ws/install/setup.bash

# Terminal 1
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true slam:=true

# Terminal 4
ros2 launch explore_lite explore.launch.py use_sim_time:=true
```

---

## 5. Verification Checklist

After all four are running:

| Check | Command / observation |
|-------|------------------------|
| `/map` | `ros2 topic echo /map --once` → OccupancyGrid, frame_id=map |
| `/global_costmap/costmap` | `ros2 topic list \| grep global_costmap` → topic exists |
| `/local_costmap/costmap` | `ros2 topic list \| grep local_costmap` → topic exists |
| `navigate_to_pose` | `ros2 action list` → navigate_to_pose present |
| `/explore/status` | `ros2 topic echo /explore/status` → ExploreStatus messages |
| Autonomous motion | Robot drives to frontiers and updates the map without you sending goals in Nav2 |

---

## 6. Common Issue: Mixed SLAM and Localization Mode

**Wrong setup:**  
- **Cartographer** (or another SLAM) publishing `/map` and TF.  
- **Nav2** started in **localization mode** (AMCL + map_server loading a static map).  
- Two different “sources of truth” for the map and pose: SLAM vs AMCL + static map.

**Why manual goals could still work:**  
- You send a goal in the map frame; Nav2 and AMCL can still plan and execute if the static map and TF are roughly consistent, so manual `navigate_to_pose` may succeed.

**Why autonomous exploration failed or was unreliable:**  
- explore_lite uses the **live** map (e.g. from slam_toolbox) to find frontiers and sends goals in that frame.  
- If Nav2/AMCL is using a **different** map (static) or different TF source, frontiers and goals are in the wrong frame or inconsistent with what Nav2 believes, so the robot may not go to the right place, get stuck, or behave erratically.

**Correct fix:**  
- Use **one** mapping/localization path: **Nav2 with slam:=true** (or equivalent), so that Nav2 uses the **same** live `/map` and TF as explore_lite (from slam_toolbox). No AMCL, no static map_server, no separate Cartographer in the same graph.

---

## 7. Notes

- **return_to_init:** If enabled in explore_lite params, the robot is intended to return to the start when exploration is “finished”. In practice, “finished” is determined by frontier exhaustion; depending on the environment and parameters, frontiers may remain for a long time, so the return-home behavior may not trigger obviously or soon.
- For more detail on run order, parameters, and troubleshooting, see `TB3_EXPLORATION_RUN_ORDER.md` and `EXPLORE_LITE_TB3_CONFIG.md` in this folder.
