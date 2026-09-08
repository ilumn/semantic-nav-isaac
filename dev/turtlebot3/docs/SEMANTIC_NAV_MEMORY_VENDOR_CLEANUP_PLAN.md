# Semantic Nav Memory Vendor Cleanup Plan

This cleanup makes the robot deployment cloneable from the main GitHub repo
without relying on a local nested checkout of `semantic-nav-memory`.

## Problem

The ROS refiner package currently depends on a sibling checkout at repo root:

```text
semantic-nav-memory/
```

That directory is an independent Git repository and is ignored by the main repo.
If the robot clones only `quadruped-semantic-navigation`, it will not receive the
worker code needed for optional COLMAP semantic enrichment.

There is also a stale tracked gitlink at:

```text
dev/turtlebot3/ros2_ws/src/m-explore-ros2
```

The directory is empty locally, `.gitmodules` is absent, and the current stack
uses `tb3_frontier_exploration` instead.

## Goal

Make one main-repo clone sufficient for the robot's ROS stack and optional
refiner source code.

## Scope

In scope:

- Vendor the runtime-relevant tracked `semantic-nav-memory` source into an
  organized main-repo directory.
- Exclude generated files, venvs, caches, local binaries, local model downloads,
  and nested `.git` metadata.
- Update `tb3_semantic_refiner` to find the vendored worker location.
- Keep the refiner optional and disabled for the first real robot validation.
- Remove the stale `m-explore-ros2` gitlink from the tracked tree.
- Update robot deployment docs/checklists so the robot no longer needs a second
  manual clone for `semantic-nav-memory`.

Out of scope:

- Installing robot packages.
- Deleting robot disk data.
- Enabling autonomous motion.
- Making COLMAP mandatory for navigation.

## Proposed Layout

Vendor source here:

```text
dev/turtlebot3/external/semantic-nav-memory/
```

Reasons:

- It is clearly scoped to the TurtleBot3 deployment.
- It avoids polluting the ROS workspace `src/` with a non-ROS Python package.
- It keeps third-party/runtime support code near the robot integration docs.
- It lets `tb3_semantic_refiner` locate the worker relative to the repo root.

## Files To Vendor

Vendor only files tracked by the nested repo at commit:

```text
c5ad423 Improve COLMAP reconstruction pipeline
```

Important tracked content includes:

- `pyproject.toml`
- `semantic_nav_memory/`
- `assets/models/README.md`
- `assets/models/MobileNetSSD_deploy.prototxt`
- `docs/system-report.md`
- `README.md`
- tests

Do not vendor:

- `.git/`
- `.venv/`
- `.venv-cpu/`
- `.local-bin/`
- `.pytest_cache/`
- `.ultralytics/`
- `semantic_nav_memory.egg-info/`
- `__pycache__/`
- upstream smoke/demo media that is not needed by the robot worker

## Refiner Lookup Strategy

`tb3_semantic_refiner.worker_core` should prefer the vendored path:

```text
dev/turtlebot3/external/semantic-nav-memory
```

It may keep the old root sibling path as a fallback for developer convenience:

```text
semantic-nav-memory
```

Runtime behavior should remain the same:

- If the worker is disabled, missing worker dependencies must not affect live
  navigation.
- If the worker is enabled but the vendored code is missing or dependencies are
  missing, the refiner job should fail gracefully and publish status.
- Live semantic navigation remains authoritative.

## Deployment Impact

After this cleanup, the robot can fetch one main repository branch and get:

- ROS packages
- real robot launch files
- semantic map/refiner packages
- vendored `semantic-nav-memory` worker source
- robot install plan/checklist

The robot will still need runtime dependencies such as Python packages,
`ffmpeg`, `colmap`, and `pycolmap` before enabling the refiner worker.

## Validation

Required validation:

1. `git diff --check`
2. Confirm no generated venv/cache/local binary files are staged.
3. Confirm `m-explore-ros2` gitlink is gone.
4. Run focused `tb3_semantic_refiner` worker command unit tests if local deps
   allow them.
5. Confirm docs reference the vendored path instead of requiring a second clone.
