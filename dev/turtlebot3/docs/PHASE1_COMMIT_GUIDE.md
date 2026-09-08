# Phase 1 Baseline — Git Packaging & Commit Guide

> Historical baseline: Gazebo commands in this document predate the Isaac Sim
> sibling port and are not current runtime instructions.

Clean staging and commit plan for TurtleBot3 autonomous exploration Phase 1.

---

## 1. Files to include (Phase 1 work)

| Path | Purpose |
|------|--------|
| `dev/turtlebot3/README.md` | Already tracked |
| `dev/turtlebot3/setup_ros2_ws.sh` | Create ROS2 workspace |
| `dev/turtlebot3/migrate_workspace.sh` | Migrate m-explore-ros2 from legacy ws |
| `dev/turtlebot3/ros2_ws/build.sh` | Build workspace (system Python, colcon) |
| `dev/turtlebot3/ros2_ws/build_tb3_frontier.sh` | Build only `tb3_frontier_exploration` (documented in README) |
| `dev/turtlebot3/ros2_ws/enter_tb3_dev.sh` | Source dev environment |
| `dev/turtlebot3/config/explore_lite_params.yaml` | TurtleBot3 explore_lite config |
| `dev/turtlebot3/docs/EXPLORE_LITE_TB3_CONFIG.md` | explore_lite config notes |
| `dev/turtlebot3/docs/TB3_AUTONOMOUS_EXPLORATION_PLAN.md` | Phase 1 & 2 implementation plan |
| `dev/turtlebot3/docs/TB3_EXPLORATION_RUN_ORDER.md` | Launch order and verification checklist |
| `dev/turtlebot3/docs/PHASE1_COMMIT_GUIDE.md` | This commit guide (optional) |
| `.gitignore` | Existing + ROS2 ws artifacts (already has turtlebot3 install/log) |

**Removed (optional diagnostics):** `diagnose_workspace.sh`, `check_exploration_prereqs.sh`, `validate_exploration_baseline.sh` — redundant with manual checks and `colcon build`. **Removed earlier:** `build_with_system_python.sh` (redundant with `build.sh`).

---

## 2. Do NOT add

| Path / pattern | Reason |
|----------------|--------|
| `dev/turtlebot3/ros2_ws/build/` | Colcon build output (already ignored by root `build/`) |
| `dev/turtlebot3/ros2_ws/install/` | Colcon install (in .gitignore) |
| `dev/turtlebot3/ros2_ws/log/` | Colcon logs (in .gitignore) |
| `dev/turtlebot3/ros2_ws/src/` | m-explore-ros2: use migrate_workspace.sh from legacy ws; contains .git / large tree |
| `dev/turtlebot3/ros2_ws/.build_used_system_python` | Build marker (in .gitignore) |

---

## 3. .gitignore recommendations

Current `.gitignore` already contains:

- `dev/turtlebot3/ros2_ws/install/`
- `dev/turtlebot3/ros2_ws/log/`
- `dev/turtlebot3/ros2_ws/.build_used_system_python`
- Root `build/` (covers `ros2_ws/build/`)

Optional (for clarity):

- Add `dev/turtlebot3/ros2_ws/build/` explicitly under the ROS2 workspace section if you want it spelled out.

No change strictly required for Phase 1.

---

## 4. Exact git commands

Run from the repository root.

```bash
# 1. Status
git status

# 2. Stage Phase 1 files (scripts, config, docs; record deletion)
git add dev/turtlebot3/config/explore_lite_params.yaml
git add dev/turtlebot3/docs/EXPLORE_LITE_TB3_CONFIG.md
git add dev/turtlebot3/docs/TB3_AUTONOMOUS_EXPLORATION_PLAN.md
git add dev/turtlebot3/docs/TB3_EXPLORATION_RUN_ORDER.md
git add dev/turtlebot3/ros2_ws/build.sh
git add dev/turtlebot3/ros2_ws/build_tb3_frontier.sh
git add dev/turtlebot3/ros2_ws/enter_tb3_dev.sh
# Optional: include this guide in the commit
# git add dev/turtlebot3/docs/PHASE1_COMMIT_GUIDE.md

# 3. Verify staging (no src/, build/, install/, log/)
git status

# 4. Commit
git commit -m "Phase 1: TurtleBot3 autonomous exploration baseline (Gazebo + slam_toolbox + Nav2 slam mode + explore_lite)"

# 5. Push (current branch: feature/slam-automation)
git push origin feature/slam-automation
```

Note: `git add dev/turtlebot3/ros2_ws/build_with_system_python.sh` on a **deleted** file stages the deletion. If the file is already gone, `git add -u dev/turtlebot3/` or `git add dev/turtlebot3/ros2_ws/` will also stage the deletion.

---

## 5. Branch name

You are already on **feature/slam-automation**, which fits Phase 1 (SLAM + automation with explore_lite). Recommendation: **keep it** and push there.

Alternative if you want Phase 1–specific: **feature/tb3-exploration-phase1** (create and push that branch instead of feature/slam-automation).

---

## 6. Suggested commit message (one line)

```
Phase 1: TurtleBot3 autonomous exploration baseline (Gazebo + slam_toolbox + Nav2 slam mode + explore_lite)
```

Extended (for `git commit -m "..."` body or editor):

```
Phase 1: TurtleBot3 autonomous exploration baseline

- ROS2 workspace setup, migrate m-explore-ros2, build/enter/diagnose scripts
- explore_lite params and docs (config, run order, Phase 2 plan)
- Prereq and baseline validation scripts
- Correct baseline: Gazebo + slam_toolbox + Nav2 (slam:=true) + explore_lite
- Drop mixed SLAM/localization setup (no AMCL/map_server/Cartographer in same graph)
- Remove redundant build_with_system_python.sh
```

---

## 7. Release-style summary (PR / commit description)

Use this in the PR body or as the commit description:

---

**Phase 1: TurtleBot3 autonomous exploration baseline**

This adds a clean TurtleBot3 development area and a validated Phase 1 baseline for autonomous frontier exploration in ROS2 Humble.

**Delivered**

- **Workspace:** New ROS2 workspace under `dev/turtlebot3/ros2_ws` with setup, migration, build, and dev-environment scripts; m-explore-ros2 migrated and built from legacy workspace.
- **Integration:** Gazebo simulation, slam_toolbox, Nav2 (with `slam:=true`), and explore_lite verified and documented.
- **Demonstration:** Autonomous exploration in simulation without manually sending goals; robot drives to frontiers and builds the map.
- **Config & docs:** TurtleBot3-focused explore_lite params, config notes, run-order checklist, and Phase 2 implementation plan.
- **Validation:** Scripts to check exploration prereqs and baseline readiness (topics, actions, packages).

**Architecture (important)**

- The correct Phase 1 stack is **Gazebo + slam_toolbox + Nav2 (slam:=true) + explore_lite**. Nav2 is run in SLAM mode (no separate localization).
- A previous mixed setup (Cartographer + Nav2 in localization mode with AMCL/map_server) was inconsistent and made autonomous exploration unreliable; manual Nav2 goals could still work. Phase 1 does not mix SLAM and localization in the same graph.

**Not committed**

- `ros2_ws/build/`, `install/`, `log/` (ignored).
- `ros2_ws/src/m-explore-ros2` (obtained via `migrate_workspace.sh` from legacy workspace).

---

Use the commands in **Section 4** as the single source of truth for staging and pushing.
