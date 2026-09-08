# `tb3_memory`

`tb3_memory` stabilizes robot-relative localized object observations into the
existing `vision_msgs/msg/Detection3DArray` compatibility stream. It is a pure
ROS component and requires no simulator-specific code.

## Interfaces

- input: `/localizer_node/localized_objects`;
- output: `/semantic_memory_node/objects`;
- frame: `base_link` for this compatibility layer.

Matching uses semantic label plus a configurable Euclidean-distance threshold,
then applies exponential position smoothing and stale/removal timeouts. The
persistent map-frame authority remains `semantic_map_memory_node` on
`/semantic_map/state`.

Names come from `semantic_targets.yaml`; semantic nodes use only
`semantic_name`, `detector_label`, aliases, and `enabled`. `isaac_asset` is
scene metadata and is ignored at runtime.

Use `dev/isaac_sim/run_stack.sh` for supported operation. For component
debugging:

```bash
ros2 launch tb3_memory semantic_memory.launch.py use_sim_time:=true
ros2 topic echo /semantic_memory_node/objects
```
