# explore_lite Configuration for TurtleBot3

Based on inspection of `ros2_ws/src/m-explore-ros2`.

---

## 1. Launch files

| Package   | Launch file                    | Purpose |
|----------|---------------------------------|--------|
| **explore_lite** | `explore/launch/explore.launch.py` | Starts `explore_node` with `config/params.yaml`, `use_sim_time` and optional `namespace`. |
| map_merge | `map_merge/launch/map_merge.launch.py` | Multi-robot map merging (not needed for single-robot exploration). |
| map_merge | `map_merge/launch/from_map_server.launch.py` | Map merge from map_server. |

For single-robot exploration you only need **explore_lite** → `explore/launch/explore.launch.py`.

---

## 2. Config / params files

| File | Use case |
|------|----------|
| **explore/config/params.yaml** | Map from SLAM: subscribes to `map` and `map_updates`. Loaded by default in `explore.launch.py`. |
| **explore/config/params_costmap.yaml** | Map from Nav2 global costmap: subscribes to `global_costmap/costmap` and `global_costmap/costmap_updates`. |

The launch file does **not** load `params_costmap.yaml` by default; to use it you must pass that file as the node config (e.g. custom launch or parameter override).

---

## 3. Topic names explore_lite expects by default

From **explore** node (backed by `costmap_client`):

| Parameter | Code default | params.yaml | params_costmap.yaml |
|-----------|----------------|-------------|----------------------|
| `costmap_topic` | `costmap` | `map` | `/global_costmap/costmap` |
| `costmap_updates_topic` | `costmap_updates` | `map_updates` | `/global_costmap/costmap_updates` |
| `robot_base_frame` | `base_link` | `base_link` | `base_link` |
| `transform_tolerance` | `0.3` | `0.3` | `0.3` |

Other parameters (explore node): `planner_frequency`, `progress_timeout`, `potential_scale`, `orientation_scale`, `gain_scale`, `min_frontier_size`, `return_to_init`, `visualize`.

- **Global frame** is taken from the costmap message `header.frame_id` (e.g. `map`).
- **Robot pose** is computed via TF from `robot_base_frame` to that global frame.

---

## 4. `/map` vs `/global_costmap/costmap`

| Source | Topic | Pros | Cons |
|--------|--------|------|------|
| **SLAM (e.g. slam_toolbox)** | `/map` | Same frame as your TF; simple; no inflation. | No obstacle inflation (only raw occupancy). |
| **Nav2 global costmap** | `/global_costmap/costmap` | Inflated obstacles; consistent with Nav2 planner. | Depends on Nav2 and costmap layers; frame is still usually `map`. |

**Recommendation for TurtleBot3:**

- **Prefer `/map`** when you run SLAM (e.g. slam_toolbox) and want minimal setup; ensure TF `map` → `odom` → `base_link` and that `/map` is published.
- Use **`/global_costmap/costmap`** if you already run Nav2 and want frontiers to respect inflated obstacles (fewer goals in narrow passages).

---

## 5. robot_base_frame for TurtleBot3

TurtleBot3 uses **`base_link`** as the robot base frame. Both `params.yaml` and `params_costmap.yaml` already set:

- **robot_base_frame: base_link**

No change needed for a standard TurtleBot3.

---

## 6. map_updates / costmap_updates — required or optional?

- The node **always subscribes** to `costmap_updates_topic` (e.g. `map_updates` or `global_costmap/costmap_updates`).
- **Full map** is applied via `costmap_topic` (OccupancyGrid); **partial updates** are applied via the updates topic (OccupancyGridUpdate).
- If nothing publishes on the updates topic, the callback simply never runs; the internal costmap still updates from full messages on `costmap_topic`.

So in practice **map_updates is optional**: exploration works with only full map updates. Partial updates are an optimization (faster updates when only a region changes).

- **slam_toolbox**: typically publishes `/map` only (no `/map_updates`). Use `costmap_topic: map` and e.g. `costmap_updates_topic: map_updates`; the subscription will stay idle and full map updates are enough.
- **Nav2 global costmap**: often publishes `/global_costmap/costmap_updates`; then use `params_costmap.yaml` or the same topic names for both costmap and costmap_updates.

---

## 7. Recommended parameter mapping for TurtleBot3

Use one of the two setups below.

### Option A — SLAM only (e.g. slam_toolbox, no Nav2 costmap)

Use **explore/config/params.yaml** as loaded by **explore.launch.py** (default). No change needed if you keep the repo defaults.

### Option B — Use Nav2 global costmap

Use **explore/config/params_costmap.yaml** (or equivalent values). Either launch explore with that file as the config, or override parameters so they match.

---

## 8. Exact suggested topic/frame values (TurtleBot3)

| Parameter | Option A (SLAM `/map`) | Option B (Nav2 costmap) |
|-----------|-------------------------|---------------------------|
| **robot_base_frame** | `base_link` | `base_link` |
| **costmap_topic** | `map` | `/global_costmap/costmap` |
| **costmap_updates_topic** | `map_updates` | `/global_costmap/costmap_updates` |
| **transform_tolerance** | `0.3` | `0.3` |
| **planner_frequency** | `0.15` | `0.15` |
| **progress_timeout** | `30.0` | `30.0` |
| **min_frontier_size** | `0.75` | `0.5` (or match A) |
| **return_to_init** | `true` or `false` | same |
| **visualize** | `true` | `true` |

**Frames:** With Option A, global frame is `map` (from `/map` header). With Option B it is usually still `map` (Nav2 global costmap typically uses `map`). Ensure TF: `map` → `odom` → `base_link`.

**Launch:** Default `explore.launch.py` loads `params.yaml` (Option A). For Option B, use a launch that passes `params_costmap.yaml` to the explore node, or override the two costmap topic parameters in your launch/parameter file.
