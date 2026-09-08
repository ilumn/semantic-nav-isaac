# Semantic Nav Memory Vendor Cleanup Checklist

This checklist corresponds to
`SEMANTIC_NAV_MEMORY_VENDOR_CLEANUP_PLAN.md`.

## 1. Planning

- [x] Identify that `semantic-nav-memory/` is a nested Git repo.
- [x] Confirm `semantic-nav-memory/` is ignored by the main repo.
- [x] Confirm the nested repo is clean.
- [x] Confirm nested repo commit `c5ad423` contains the needed worker changes.
- [x] Identify stale `m-explore-ros2` gitlink.
- [x] Save cleanup plan.
- [x] Save cleanup checklist.

## 2. Vendor Source

- [x] Create `dev/turtlebot3/external/semantic-nav-memory/`.
- [x] Copy runtime-relevant files tracked by the nested `semantic-nav-memory`
  repo.
- [x] Exclude `.git/`.
- [x] Exclude `.venv/` and `.venv-cpu/`.
- [x] Exclude `.local-bin/`.
- [x] Exclude `.pytest_cache/`.
- [x] Exclude `.ultralytics/`.
- [x] Exclude `semantic_nav_memory.egg-info/`.
- [x] Exclude Python `__pycache__/`.
- [x] Exclude upstream smoke/demo media that is not needed by the robot worker.
- [x] Confirm vendored `pyproject.toml` exists.
- [x] Confirm vendored `semantic_nav_memory/cli.py` exists.
- [x] Confirm vendored `semantic_nav_memory/geometry_colmap.py` exists.
- [x] Confirm vendored tests exist.

## 3. Refiner Integration

- [x] Update `tb3_semantic_refiner.worker_core` to prefer the vendored path.
- [x] Keep root-level `semantic-nav-memory/` as a developer fallback if present.
- [x] Update worker error message to list searched paths.
- [x] Confirm `PYTHONPATH` points at the selected worker root.
- [x] Confirm `.local-bin` PATH injection still works if a selected worker root
  has local binaries.

## 4. Remove Legacy Gitlink

- [x] Confirm no runtime launch depends on `m-explore-ros2`.
- [x] Remove tracked `dev/turtlebot3/ros2_ws/src/m-explore-ros2` gitlink.
- [x] Confirm `git submodule status` no longer errors because of that path.

## 5. Documentation

- [x] Update real robot runbook to describe vendored worker source.
- [x] Update Jetson setup guide to avoid requiring a second manual clone.
- [x] Update Yahboom install plan.
- [x] Update Yahboom install checklist.
- [x] Keep first robot launch instructions with `refiner_worker_enabled:=false`.

## 6. Validation

- [x] Run `git diff --check`.
- [x] Run a file listing check for accidental venv/cache/local binary staging.
- [x] Run focused `tb3_semantic_refiner` worker tests if dependencies allow.
- [x] Confirm main repo status contains expected new/modified/deleted files.
- [x] Report any remaining deployment blockers.
