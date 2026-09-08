from __future__ import annotations

import argparse
from pathlib import Path

from .config import PipelineConfig, configure_environment
from .pipeline import run_pipeline


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="semantic-nav-memory-worker",
        description="Run the semantic-nav-memory pipeline without the web server.",
    )
    parser.add_argument("--job-id", required=True)
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--video-path")
    input_group.add_argument("--image-dir")
    parser.add_argument("--frame-manifest")
    parser.add_argument("--job-dir", required=True)
    parser.add_argument("--frame-sample-fps", type=float, default=1.0)
    parser.add_argument("--max-keyframes", type=int, default=128)
    parser.add_argument("--max-image-size", type=int, default=1440)
    parser.add_argument("--yolo-model-size", default="")
    parser.add_argument("--yolo-model-name", default="")
    parser.add_argument("--yolo-confidence", type=float, default=0.18)
    parser.add_argument("--prompt-preset", default="")
    parser.add_argument("--scene-profile", default="general")
    parser.add_argument("--camera-fx", type=float)
    parser.add_argument("--camera-fy", type=float)
    parser.add_argument("--camera-cx", type=float)
    parser.add_argument("--camera-cy", type=float)
    parser.add_argument("--camera-width", type=int)
    parser.add_argument("--camera-height", type=int)
    parser.add_argument("--prompt", action="append", default=[])
    return parser


def main() -> None:
    args = _parser().parse_args()
    configure_environment()

    config_kwargs = dict(
        frame_sample_fps=max(0.5, float(args.frame_sample_fps)),
        max_keyframes=max(8, int(args.max_keyframes)),
        max_image_size=max(256, int(args.max_image_size)),
        yolo_confidence=float(args.yolo_confidence),
        scene_profile=str(args.scene_profile or "general"),
    )
    for field_name in (
        "camera_fx",
        "camera_fy",
        "camera_cx",
        "camera_cy",
        "camera_width",
        "camera_height",
    ):
        value = getattr(args, field_name)
        if value is not None:
            config_kwargs[field_name] = value
    if args.yolo_model_size:
        config_kwargs["yolo_model_size"] = str(args.yolo_model_size)
    if args.yolo_model_name:
        config_kwargs["yolo_model_name"] = str(args.yolo_model_name)
    if args.prompt_preset:
        config_kwargs["prompt_preset"] = str(args.prompt_preset)
    if args.prompt:
        config_kwargs["prompt_vocabulary"] = list(args.prompt)

    config = PipelineConfig(**config_kwargs)

    def progress(step: str, value: float) -> None:
        print(f"{step}:{value:.3f}", flush=True)

    run_pipeline(
        job_id=str(args.job_id),
        job_dir=Path(args.job_dir),
        config=config,
        progress=progress,
        video_path=Path(args.video_path) if args.video_path else None,
        image_dir=Path(args.image_dir) if args.image_dir else None,
        frame_manifest_path=Path(args.frame_manifest) if args.frame_manifest else None,
    )


if __name__ == "__main__":
    main()
