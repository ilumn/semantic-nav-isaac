# `tb3_semantic_refiner`

`tb3_semantic_refiner` is the asynchronous bridge between the live ROS2 semantic-map stack and the standalone `semantic-nav-memory` worker.

Current implementation status:

- `semantic_refiner_node` keeps a rolling on-disk bundle of sampled RGB frames and synchronized metadata
- jobs are launched asynchronously in a background thread, not on the live ROS callback path
- the node invokes `semantic-nav-memory` through a non-browser subprocess worker entrypoint
- completed jobs export pose correspondences, alignment metadata, reference-only RViz markers, and a refined semantic state topic

## Topics

Inputs:

- `/camera/image_raw`
- `/camera/camera_info`
- `/map`
- `/tf`

Outputs:

- `/semantic_map/refiner_status`
- `/semantic_map/colmap_markers`
- `/semantic_map/refined_state`

## Runtime Expectations

- The worker source is vendored at
  `dev/turtlebot3/external/semantic-nav-memory`.
- A root-level `semantic-nav-memory/` checkout is still supported as a developer
  override, but it is not required for robot deployment.
- `worker_python_executable` should point at a Python `3.12` environment that has `semantic-nav-memory` dependencies installed.
- The external worker expects `ffmpeg`, `ffprobe`, `pycolmap`, and `ultralytics` runtime dependencies.
- `worker_prompt_preset`, `worker_scene_profile`, `worker_yolo_model_size`, and `worker_prompt_vocabulary` are configurable from ROS params.

## Authority Boundary

- The refinement layer is reference-only.
- Weak or rejected alignments are not published as overlays.
- The live semantic map remains authoritative for navigation.
- Nothing in this package writes back into SLAM `/map`.
- Previously accepted refinement jobs are not replayed when the node starts; overlays appear only from jobs accepted during the current run.

## Debugging

- Use `/semantic_map/refiner_status` for job lifecycle, timeout, and alignment diagnostics.
- Use `/semantic_map/colmap_markers` in RViz for the aligned reconstruction-derived overlay.
- Use `/semantic_map/refined_state` to inspect the aligned semantic output as machine-readable ROS messages.

## Launch

```bash
ros2 launch tb3_semantic_refiner semantic_refiner.launch.py
```
