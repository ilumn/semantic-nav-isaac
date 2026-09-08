# `tb3_detector`

The detector runs YOLOv8 on `/camera/image_raw` and publishes
`vision_msgs/msg/Detection2DArray` plus an annotated debug image. It consumes
rendered RGB like any real camera; no Isaac ground-truth detections are used.

## Interfaces

| Direction | Topic | Type |
| --- | --- | --- |
| input | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| input | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| output | `/detector_node/detections` | `vision_msgs/msg/Detection2DArray` |
| output | `/detector_node/debug_image` | `sensor_msgs/msg/Image` |

The vendored `models/yolov8n.pt` weight is required. The Isaac launch defaults
to `device:=cuda:0`. The node itself supports `detector_device:=cpu` for
isolated/manual launches; the supported `dev/isaac_sim/run_stack.sh` workflow
intentionally requires the validated NVIDIA CUDA runtime.

The canonical label mapping is:

| Semantic name | Detector label | Isaac asset key |
| --- | --- | --- |
| `table` | `bench` | `table_marble` |
| `person` | `person` | `person_standing` |
| `stop_sign` | `stop sign` | `stop_sign` |

The full rendered-scene validation is deliberately separate from the previous
simulator baseline: object appearance, illumination, and camera exposure must be
rechecked in Isaac before detection quality is considered accepted.

## Run

Use `dev/isaac_sim/run_stack.sh` for the supported full stack. For isolated
debugging after sourcing the Jazzy workspace:

```bash
ros2 launch tb3_detector detector.launch.py use_sim_time:=true device:=cuda:0
ros2 topic hz /detector_node/detections
```
