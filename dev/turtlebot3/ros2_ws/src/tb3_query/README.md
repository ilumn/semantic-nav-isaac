# `tb3_query`

`tb3_query` resolves a natural semantic command against the live semantic map.
Its default `semantic_map_state` mode reads `/semantic_map/state` and publishes
the selected target for the navigation adapter.

## Naming

| Field | Purpose | Example |
| --- | --- | --- |
| `semantic_name` | user-facing canonical name | `table` |
| `detector_label` | exact YOLO class | `bench` |
| `isaac_asset` | simulator-only scene key | `table_marble` |

Only enabled targets are queryable. Aliases normalize phrases such as “go to
the stop sign” to the canonical `stop_sign`; unsupported or currently unseen
targets fail without inventing a goal.

## Interfaces

- command: `/semantic_query_node/command` (`std_msgs/msg/String`);
- semantic state: `/semantic_map/state`;
- selected target: `/semantic_query_node/selected_target`;
- status: `/semantic_query_node/query_status`.

Example after the full Isaac/ROS stack is running:

```bash
ros2 topic pub --once /semantic_query_node/command \
  std_msgs/msg/String "{data: 'go to the person'}"
```

The previous simulator's smoke results are only a regression baseline. Isaac
acceptance requires this command to resolve an object observed in rendered RGB,
then reach the real Nav2 action path.
