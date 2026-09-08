# Semantic Nav Memory

Standalone monocular semantic mapping from RGB video, with sparse 3D reconstruction, open-vocabulary detection, semantic fusion, and a browser inspector.

## What It Does

1. Upload a prerecorded RGB video.
2. Extract and subsample keyframes with `ffmpeg`.
3. Reconstruct camera poses and a sparse point cloud with `pycolmap`.
4. Run `YOLO-World` over those keyframes with a scene-aware prompt vocabulary.
5. Ground detections into the reconstruction, fuse them into persistent entities, and infer places and relations.
6. Inspect the result in the browser through 3D, video, and graph views.

## Repository Layout

```text
semantic_nav_memory/   Core pipeline, API, schemas, ontology, and prompts
frontend/              Static browser UI
docs/                  Plan, system report, and implementation checklist
examples/              Small tracked sample inputs
assets/models/         Local model weights directory (ignored except README)
data/                  Runtime outputs and local working data
```

## Requirements

- Python `>3.12`
- `ffmpeg` and `ffprobe` available on `PATH`
- Enough CPU/RAM for `pycolmap` and `YOLO-World`

## Setup

```bash
python3.12 -m venv .venv
source .venv/bin/activate
pip install -e .
```

If you already use the local project environment, `.venv-models` still works.

## Run
```bash
uvicorn semantic_nav_memory.server:app --host 127.0.0.1 --port 8000
```

Then open `http://127.0.0.1:8000`.

## Headless Worker

The package now also exposes a non-browser worker entrypoint for ROS-side or batch integration:

```bash
semantic-nav-memory-worker \
  --job-id demo_job \
  --video-path /abs/path/input.mp4 \
  --job-dir /abs/path/job_dir
```

The worker also accepts a direct image-sequence path, which is the preferred mode for robot-side integration because it preserves the original sampled camera frames:

```bash
semantic-nav-memory-worker \
  --job-id demo_job \
  --image-dir /abs/path/bundle/frames \
  --frame-manifest /abs/path/bundle/manifest.json \
  --job-dir /abs/path/job_dir
```

Useful worker overrides:

- `--frame-sample-fps`
- `--max-keyframes`
- `--max-image-size`
- `--camera-fx`, `--camera-fy`, `--camera-cx`, `--camera-cy`
- `--yolo-model-size`
- `--yolo-model-name`
- `--yolo-confidence`
- `--prompt-preset`
- `--scene-profile`
- repeated `--prompt`

This worker writes JSON artifacts directly under `<job_dir>/outputs/` and does not require the FastAPI server.

## Model Weights

The UI exposes `YOLO-World` size options: `small`, `medium`, `large`, and `xlarge`.

- Local checkpoints can live in `assets/models/`
- Ultralytics-managed weights can also live in `.ultralytics/weights/`
- Downloaded weights are ignored by git

## Output Artifacts

Each job writes results under `data/jobs/<job_id>/outputs/`:

- `video_metadata.json`
- `reconstruction.json`
- `detections.json`
- `tracklets.json`
- `observations.json`
- `entities.json`
- `places.json`
- `relations.json`
- `task_anchors.json`
- `scene_graph.json`

## Notes

- Reconstruction scale is relative.
- The aligned world frame is PCA-normalized for inspection, not gravity-calibrated.
- Semantic grounding keeps only detections with supporting sparse reconstruction points.
- The worker runtime is expected to run under Python `3.12` with the package dependencies installed.
