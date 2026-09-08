# `tb3_localizer`

`tb3_localizer` fuses YOLO image detections with a planar `LaserScan` and
publishes robot-relative object points. It is simulator-neutral and runs against
the stable Isaac interfaces `/camera/image_raw` and `/scan`.

## Pipeline

For each `vision_msgs/Detection2D`:

1. convert the bounding-box centre pixel to a horizontal camera bearing;
2. find the matching LiDAR sample;
3. take the median valid range in a configurable scan window;
4. convert range/bearing to `(x, y)` in `base_link`;
5. publish `geometry_msgs/PointStamped` on
   `/localizer_node/object_points`.

Image left maps to ROS `+Y`; image centre maps to robot `+X`. The camera is
configured for a 62.2 degree horizontal field of view.

## Scan compatibility

The core accepts both common full-scan layouts:

- `[0, 2π)`, used by the previous simulator;
- `[-π, π]`, commonly produced by Isaac/ROS LiDAR publishers.

It resolves equivalent angles only when they fall inside the published scan,
so a bearing outside a partial field of view is rejected. Median windows wrap
across the first/last ray for full 360 degree scans.

## Topics

| Direction | Topic | Type |
| --- | --- | --- |
| input | `/detector_node/detections` | `vision_msgs/msg/Detection2DArray` |
| input | `/scan` | `sensor_msgs/msg/LaserScan` |
| input | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| output | `/localizer_node/object_points` | `geometry_msgs/msg/PointStamped` |

Sensor subscriptions use sensor-data QoS so they are compatible with Isaac
Sim's best-effort image and LiDAR publishers.

## Semantic target mapping

| Semantic name | YOLO label | Isaac asset key |
| --- | --- | --- |
| `table` | `bench` | `table_marble` |
| `person` | `person` | `person_standing` |
| `stop_sign` | `stop sign` | `stop_sign` |

Ground-truth Isaac labels never feed this node; RGB detection plus LiDAR remains
the authoritative localization path.

## Run

The supported path starts this node through the full stack:

```bash
dev/isaac_sim/run_sim.sh
dev/isaac_sim/run_stack.sh
```

For component debugging after sourcing the Jazzy workspace:

```bash
ros2 launch tb3_localizer localizer.launch.py use_sim_time:=true
ros2 topic hz /scan
ros2 topic echo /localizer_node/object_points
```

The implementation is intentionally planar: it does not infer a full 3D pose,
and a visual detection without a valid same-bearing LiDAR return is rejected.
