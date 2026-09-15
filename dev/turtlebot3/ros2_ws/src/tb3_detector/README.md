# `tb3_detector`

The detector runs NVIDIA LocateAnything-3B on `/camera/image_raw` and publishes
`vision_msgs/msg/Detection2DArray` plus an annotated debug image. It consumes
rendered RGB like any real camera; no Isaac ground-truth detections are used.

## Interfaces

| Direction | Topic | Type |
| --- | --- | --- |
| input | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| input | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| output | `/detector_node/detections` | `vision_msgs/msg/Detection2DArray` |
| output | `/detector_node/debug_image` | `sensor_msgs/msg/Image` |

The model is an open-vocabulary box detector. `class_filter` is the category
query sent on each frame, rather than a post-inference COCO filter. The default
query uses the scene's canonical labels:

| Semantic name | Detector label | Isaac asset key |
| --- | --- | --- |
| `table` | `bench` | `table_marble` |
| `person` | `person` | `person_standing` |
| `stop_sign` | `stop sign` | `stop_sign` |

Locate Anything does not return calibrated confidence values or tracking IDs.
The ROS adapter therefore publishes a score of `1.0` for every returned box and
leaves `Detection2D.id` empty. The score means that the model returned the box;
it is not a probability.

The model revision is pinned in `detector.yaml` and the Isaac runtime helpers.
The node defaults to `local_files_only: true`, so startup cannot silently fetch
different code or weights. Install dependencies and fetch the pinned snapshot:

```bash
dev/isaac_sim/bootstrap_runtime.sh --allow-download
dev/isaac_sim/fetch_models.sh --download
```

The NVIDIA model license limits the weights to research and evaluation use.
See the repository `THIRD_PARTY_NOTICES.md` before redistribution or deployment.

## Run

Use `dev/isaac_sim/run_stack.sh` for the supported full stack. For isolated
debugging after sourcing the Jazzy workspace:

```bash
ros2 launch tb3_detector detector.launch.py use_sim_time:=true device:=cuda:0
ros2 topic hz /detector_node/detections
```

`generation_mode:=hybrid` uses parallel box decoding with autoregressive
fallback. `fast`, `slow`, and `hybrid` are supported. The subscription queue is
kept at one frame because VLM inference is slower than the camera stream.
