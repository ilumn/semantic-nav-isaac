# `tb3_semantic_map`

`tb3_semantic_map` is the new live semantic-map package for TurtleBot3.

Current implementation status:

- message interfaces exist via `tb3_semantic_map_msgs`
- `semantic_map_node` samples `/camera/image_raw`, reuses the Locate Anything detector stack, and grounds detections into `map`
- the node maintains live map-frame entity memory plus derived places, relations, and anchors
- the node publishes semantic-map state, compatibility objects, markers, status, and debug images

## Topics

Inputs:

- `/camera/image_raw`
- `/camera/camera_info`
- `/scan`
- `/map`
- `/tf`

Outputs:

- `/semantic_map/state`
- `/semantic_map/compat_objects`
- `/semantic_map/markers`
- `/semantic_map/status`
- `/semantic_map/debug_image`

## Semantic State

`/semantic_map/state` publishes `SemanticMapState` in `map` frame and includes:

- live `entities`
- derived `places`
- conservative `relations`
- approach / inspection `anchors`

`/semantic_map/compat_objects` mirrors the live entities as `vision_msgs/Detection3DArray` so the existing query and nav path can consume the new semantic source without a first-pass rewrite.

## Query integration

The supported Isaac wrapper defaults to native `semantic_map_state` mode on
`/semantic_map/state`. For compatibility testing, pass these arguments through
`dev/isaac_sim/run_stack.sh`:

```bash
dev/isaac_sim/run_stack.sh \
  query_memory_topic:=/semantic_map/compat_objects \
  query_memory_mode:=detection3d \
  query_output_frame:=map
```

## Debugging

- Use `/semantic_map/markers` in RViz for the live semantic layer.
- Use `/semantic_map/debug_image` to inspect detector output.
- Use `/semantic_map/status` for grounding and memory heartbeat diagnostics.
- The semantic layer is navigation-facing, but it never writes back into SLAM `/map`.

## Launch

```bash
ros2 launch tb3_semantic_map semantic_map.launch.py
```
