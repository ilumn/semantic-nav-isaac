# TurtleBot3 Autonomous Exploration — Implementation Plan

Two-phase plan: baseline (m-explore-ros2) then custom package `tb3_frontier_exploration`.

---

## Phase 1: Baseline with m-explore-ros2

**Goal:** Run explore_lite, verify stack (topics, TF, SLAM, Nav2).

### 1.1 Required Stack (topics / TF / actions)

| Component | Topic / Interface | Frame / Notes |
|-----------|-------------------|---------------|
| Map | `map` (nav_msgs/OccupancyGrid) | `map` frame |
| Costmap updates | `map_updates` (map_msgs/OccupancyGridUpdate) optional | Same as map |
| Global costmap | `global_costmap/costmap` (nav_msgs/OccupancyGrid) | Alternative to `map` for explore_lite config |
| Odometry | `odom` or `odometry/filtered` (nav_msgs/Odometry) | For localization |
| TF | `map` → `odom` → `base_link` | SLAM/localization + robot_state_publisher |
| Nav2 action | `navigate_to_pose` (nav2_msgs/action/NavigateToPose) | Nav2 BT server |

### 1.2 explore_lite Parameters (params.yaml)

- `robot_base_frame`: `base_link`
- `costmap_topic`: `map` (or `global_costmap/costmap` if using Nav2 costmap)
- `costmap_updates_topic`: `map_updates`
- `planner_frequency`, `progress_timeout`, `min_frontier_size`, etc.

### 1.3 Phase 1 Verification Checklist

- [ ] **TF:** `map` → `odom` → `base_link`; `ros2 run tf2_ros tf2_echo map base_link`
- [ ] **Map:** `ros2 topic echo /map` (or `/global_costmap/costmap`) — published, frame_id=map
- [ ] **Odometry:** `ros2 topic echo /odometry/filtered` or `/odom`
- [ ] **Nav2:** `ros2 action list` shows `navigate_to_pose`; BT server running
- [ ] **explore_lite:** `ros2 run explore_lite explore` (or launch) runs; publishes `explore/status`, `explore/frontiers` (markers)
- [ ] **End-to-end:** Robot receives goals and moves; map fills; no TF/costmap errors in logs

### 1.4 Recommended Launch Order (Phase 1)

1. Robot + robot_state_publisher (URDF, TF)
2. SLAM (e.g. slam_toolbox) → publishes `map`, `map` → `odom`
3. Localization (if not using SLAM’s odom): e.g. robot_localization → `odometry/filtered`
4. Nav2 (controller, planner, BT, costmaps)
5. explore_lite (with correct `costmap_topic` and `use_sim_time` if sim)

---

## Phase 2: Custom Package `tb3_frontier_exploration`

**Goal:** Your own package with **frontier_detection_node** and **goal_assignment_node**, using `/map`, `/global_costmap/costmap`, `/odometry/filtered`, and Nav2 `NavigateToPose`.

---

### 2.1 Package Structure

```
ros2_ws/src/
  tb3_frontier_exploration/
    CMakeLists.txt
    package.xml
    README.md
    config/
      params.yaml
    include/
      tb3_frontier_exploration/
        frontier_detection.hpp
        goal_assignment.hpp
    launch/
      exploration.launch.py
      frontier_detection.launch.py
      goal_assignment.launch.py
    src/
      frontier_detection_node.cpp
      goal_assignment_node.cpp
    msg/                    # optional custom msgs
      Frontier.msg
      FrontierArray.msg
    srv/                    # optional
      GetNextGoal.srv
```

**Optional:** If you prefer reusing standard types only, you can skip `msg/` and use `geometry_msgs/PoseStamped` for goals and `geometry_msgs/PoseArray` or `visualization_msgs/MarkerArray` for frontiers.

---

### 2.2 Topic & Action Interfaces

| Direction | Topic / Action | Type | Owner |
|----------|----------------|------|--------|
| **Input (both nodes)** | `/map` | nav_msgs/OccupancyGrid | SLAM / map_server |
| **Input (both)** | `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 global costmap |
| **Input (goal_assignment)** | `/odometry/filtered` | nav_msgs/Odometry | robot_localization / odom |
| **Input (goal_assignment)** | `/frontiers` | Your FrontierArray or PoseArray / MarkerArray | frontier_detection_node |
| **Output (frontier_detection)** | `/frontiers` | FrontierArray or PoseArray / MarkerArray | frontier_detection_node |
| **Output (frontier_detection)** | `/frontiers_markers` | visualization_msgs/MarkerArray | optional debug |
| **Output (goal_assignment)** | Nav2 action | nav2_msgs/action/NavigateToPose | goal_assignment_node (client) |
| **Optional** | `/exploration_goal` | geometry_msgs/PoseStamped | goal_assignment_node (if you want to expose current goal) |

**Action:** `goal_assignment_node` is the **client** of `NavigateToPose`; Nav2 provides the action server.

---

### 2.3 Node Responsibilities

**1) frontier_detection_node**

- **Subscribes:** `/map` and optionally `/global_costmap/costmap` (configurable).
- **Publishes:** `/frontiers` (list of frontier centroids, e.g. FrontierArray or PoseArray) and optionally `/frontiers_markers` for RViz.
- **Logic:**  
  - Inflate obstacles from costmap/map (e.g. treat LETHAL/INSCRIBED as obstacles).  
  - Find frontier cells (free cell with unknown neighbor).  
  - Cluster frontier cells, compute centroid (and optionally size).  
  - Publish at a fixed rate (e.g. 1–2 Hz) or on map/costmap update.
- **Parameters:** map/costmap topic names, inflation, min cluster size, rate, frame_id.

**2) goal_assignment_node**

- **Subscribes:** `/frontiers`, `/odometry/filtered` (robot pose can be taken from odom or TF).
- **Uses:** Nav2 `NavigateToPose` action client.
- **Logic:**  
  - Choose next best frontier (e.g. information gain, distance, safety).  
  - Send goal to Nav2; on success/failure/cancel, pick next frontier and repeat.  
  - Optionally stop when no frontiers or exploration complete.
- **Parameters:** action name, frame_id (e.g. `map`), timeouts, strategy (e.g. nearest, largest, information gain).

---

### 2.4 Implementation Order

1. **Create package**  
   `ros2 pkg create tb3_frontier_exploration --build-type ament_cmake --dependencies rclcpp nav_msgs geometry_msgs visualization_msgs nav2_msgs tf2 tf2_ros std_msgs`

2. **Define interfaces (if custom)**  
   - Add `msg/Frontier.msg`, `msg/FrontierArray.msg` and generate in CMakeLists.txt, or decide to use PoseArray + MarkerArray only.

3. **Implement frontier_detection_node**  
   - Subscribe to `/map` (and optionally `/global_costmap/costmap`).  
   - Implement frontier search (grid-based).  
   - Publish `/frontiers` (+ optional markers).  
   - Test in isolation: run SLAM + Nav2, echo `/frontiers` and view markers in RViz.

4. **Implement goal_assignment_node**  
   - Subscribe to `/frontiers` and `/odometry/filtered`.  
   - Create `NavigateToPose` action client.  
   - Implement selection policy (e.g. nearest frontier in map frame).  
   - Send one goal at a time; on result, select next.  
   - Test: run full stack + frontier node, then start goal_assignment and confirm goals are sent and robot moves.

5. **Parameters & launch**  
   - Add `config/params.yaml` for both nodes.  
   - Launch files: `frontier_detection.launch.py`, `goal_assignment.launch.py`, `exploration.launch.py` (both + optional SLAM/Nav2 includes).

6. **Integration & tuning**  
   - Run with TB3 (sim or real); tune frontier min size, costmap topic, timeouts, and goal-selection strategy.

---

### 2.5 Test Checklist (Phase 2)

- [ ] **frontier_detection_node**  
  - [ ] Receives `/map` (and `/global_costmap/costmap` if used).  
  - [ ] Publishes `/frontiers` (and `/frontiers_markers`).  
  - [ ] Frontiers appear in RViz and update when map changes.

- [ ] **goal_assignment_node**  
  - [ ] Receives `/frontiers` and `/odometry/filtered`.  
  - [ ] Connects to `NavigateToPose` action server.  
  - [ ] Sends goals; robot moves; on success, next goal is sent.  
  - [ ] Handles ABORTED/CANCELED (e.g. pick another frontier or retry).

- [ ] **Frames**  
  - [ ] All poses in `map` frame; TF tree complete (map → odom → base_link).

- [ ] **End-to-end**  
  - [ ] Full pipeline runs from single launch file.  
  - [ ] Exploration progresses and coverage increases over time.

---

### 2.6 Recommended Launch File Structure

**Option A — Modular (recommended)**

- `frontier_detection.launch.py`: starts only `frontier_detection_node` (params + remappings).  
- `goal_assignment.launch.py`: starts only `goal_assignment_node`.  
- `exploration.launch.py`: includes or launches SLAM + Nav2 (or assumes they are running), then includes `frontier_detection.launch.py` and `goal_assignment.launch.py`.

**Option B — Monolithic**

- `exploration.launch.py`: starts both nodes + optionally SLAM + Nav2 in one file (simpler, less flexible).

**Suggested arguments for exploration.launch.py**

- `use_sim_time` (default true for sim).  
- `map_topic` (default `/map`).  
- `costmap_topic` (default `/global_costmap/costmap`).  
- `odom_topic` (default `/odometry/filtered`).  
- `namespace` (optional, for multi-robot later).

**Example structure (Phase 2 single launch):**

```text
exploration.launch.py
  ├── use_sim_time, map_topic, costmap_topic, odom_topic
  ├── IncludeLaunchDescription(SLAM)           # if you launch SLAM here
  ├── IncludeLaunchDescription(Nav2)           # if you launch Nav2 here
  ├── Node(frontier_detection_node)
  └── Node(goal_assignment_node)
```

---

## Summary Table

| Phase | Focus | Main outputs |
|-------|--------|--------------|
| **1** | Baseline | explore_lite running; TF, map, odom, Nav2 verified |
| **2** | Custom | Package `tb3_frontier_exploration`; frontier_detection_node + goal_assignment_node; same interfaces as above |

Use Phase 1 to validate the stack and Phase 2 to replace explore_lite with your own frontier detection and goal assignment logic while reusing `/map`, `/global_costmap/costmap`, `/odometry/filtered`, and Nav2 NavigateToPose.
