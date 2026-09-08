from __future__ import annotations

import json
import math
import subprocess
from pathlib import Path

import cv2

from .models import Keyframe, VideoMetadata


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, check=True, capture_output=True, text=True)


def probe_video(video_path: Path) -> VideoMetadata:
    result = _run(
        [
            "ffprobe",
            "-v",
            "error",
            "-print_format",
            "json",
            "-show_streams",
            "-show_format",
            str(video_path),
        ]
    )
    payload = json.loads(result.stdout)
    video_stream = next(
        stream for stream in payload["streams"] if stream.get("codec_type") == "video"
    )

    frame_rate = None
    avg_frame_rate = video_stream.get("avg_frame_rate")
    if avg_frame_rate and avg_frame_rate != "0/0":
        num, den = avg_frame_rate.split("/")
        den_value = float(den)
        if den_value:
            frame_rate = float(num) / den_value

    duration = float(video_stream.get("duration") or payload["format"].get("duration") or 0.0)
    frame_count = video_stream.get("nb_frames")

    return VideoMetadata(
        path=str(video_path),
        filename=video_path.name,
        width=int(video_stream["width"]),
        height=int(video_stream["height"]),
        duration_s=duration,
        fps=frame_rate,
        frame_count=int(frame_count) if frame_count else None,
    )


def _clear_output_dir(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale in output_dir.glob("*.jpg"):
        stale.unlink()


def _image_dimensions(image_path: Path) -> tuple[int, int]:
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"Failed to read image: {image_path}")
    height, width = image.shape[:2]
    return width, height


def _resize_dimensions(width: int, height: int, max_image_size: int) -> tuple[int, int]:
    if max_image_size <= 0 or max(width, height) <= max_image_size:
        return width, height
    if width >= height:
        scale = max_image_size / max(width, 1)
        resized_width = max_image_size
        resized_height = max(2, int(round(height * scale / 2.0)) * 2)
    else:
        scale = max_image_size / max(height, 1)
        resized_height = max_image_size
        resized_width = max(2, int(round(width * scale / 2.0)) * 2)
    return resized_width, resized_height


def _write_keyframe_image(source_path: Path, target_path: Path, max_image_size: int) -> tuple[int, int]:
    image = cv2.imread(str(source_path))
    if image is None:
        raise RuntimeError(f"Failed to read image: {source_path}")
    height, width = image.shape[:2]
    resized_width, resized_height = _resize_dimensions(width, height, max_image_size)
    if (resized_width, resized_height) != (width, height):
        image = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_AREA)
    if not cv2.imwrite(str(target_path), image, [int(cv2.IMWRITE_JPEG_QUALITY), 95]):
        raise RuntimeError(f"Failed to write keyframe image: {target_path}")
    return resized_width, resized_height


def _select_evenly_spaced(items: list[dict], max_items: int) -> list[dict]:
    if len(items) <= max_items:
        return items
    if max_items <= 1:
        return [items[0]]
    stride = (len(items) - 1) / (max_items - 1)
    return [items[min(len(items) - 1, round(index * stride))] for index in range(max_items)]


def _sequence_frames(image_dir: Path, frame_manifest_path: Path | None = None) -> list[dict]:
    if frame_manifest_path is not None and frame_manifest_path.exists():
        payload = json.loads(frame_manifest_path.read_text(encoding="utf-8"))
        manifest_frames = sorted(
            payload.get("frames", []),
            key=lambda item: (
                float(item.get("stamp_sec", 0.0)),
                int(item.get("frame_index", 0)),
            ),
        )
        if manifest_frames:
            first_stamp = float(manifest_frames[0].get("stamp_sec", 0.0))
            resolved_frames: list[dict] = []
            for index, frame in enumerate(manifest_frames):
                raw_path = frame.get("image_path")
                if not raw_path:
                    continue
                source_path = Path(str(raw_path))
                if not source_path.is_absolute():
                    source_path = (frame_manifest_path.parent / source_path).resolve()
                if not source_path.exists():
                    fallback_path = image_dir / source_path.name
                    if fallback_path.exists():
                        source_path = fallback_path
                if not source_path.exists():
                    continue
                stamp_sec = float(frame.get("stamp_sec", first_stamp))
                resolved_frames.append(
                    {
                        "source_path": source_path,
                        "frame_index": int(frame.get("frame_index", index)),
                        "timestamp_s": max(0.0, stamp_sec - first_stamp),
                    }
                )
            if resolved_frames:
                return resolved_frames

    image_paths = sorted(
        [
            path
            for pattern in ("*.jpg", "*.jpeg", "*.png")
            for path in image_dir.glob(pattern)
        ]
    )
    return [
        {
            "source_path": path,
            "frame_index": index,
            "timestamp_s": float(index),
        }
        for index, path in enumerate(image_paths)
    ]


def probe_image_sequence(image_dir: Path, frame_manifest_path: Path | None = None) -> VideoMetadata:
    frames = _sequence_frames(image_dir, frame_manifest_path)
    if not frames:
        raise RuntimeError(f"No images found under {image_dir}")
    width, height = _image_dimensions(frames[0]["source_path"])
    duration_s = float(frames[-1]["timestamp_s"]) if len(frames) > 1 else 0.0
    fps = ((len(frames) - 1) / duration_s) if duration_s > 0.0 else None
    return VideoMetadata(
        path=str(image_dir),
        filename=image_dir.name,
        width=width,
        height=height,
        duration_s=duration_s,
        fps=fps,
        frame_count=len(frames),
    )


def extract_keyframes_from_sequence(
    image_dir: Path,
    output_dir: Path,
    max_keyframes: int,
    max_image_size: int,
    frame_manifest_path: Path | None = None,
) -> list[Keyframe]:
    _clear_output_dir(output_dir)
    frames = _sequence_frames(image_dir, frame_manifest_path)
    if not frames:
        raise RuntimeError(f"No images found under {image_dir}")
    selected = _select_evenly_spaced(frames, max_items=max(1, max_keyframes))

    keyframes: list[Keyframe] = []
    for index, frame in enumerate(selected, start=1):
        output_path = output_dir / f"kf_{index:06d}.jpg"
        width, height = _write_keyframe_image(
            source_path=frame["source_path"],
            target_path=output_path,
            max_image_size=max_image_size,
        )
        keyframes.append(
            Keyframe(
                keyframe_id=f"kf_{index:03d}",
                image_name=output_path.name,
                image_path=str(output_path),
                frame_index=int(frame["frame_index"]),
                timestamp_s=float(frame["timestamp_s"]),
                width=width,
                height=height,
            )
        )
    return keyframes


def extract_keyframes(
    video_path: Path,
    output_dir: Path,
    sample_fps: float,
    max_keyframes: int,
    max_image_size: int,
) -> list[Keyframe]:
    _clear_output_dir(output_dir)

    filter_chain = f"fps={sample_fps}"
    if max_image_size > 0:
        filter_chain += (
            f",scale='if(gt(iw,ih),min({max_image_size},iw),-2)':"
            f"'if(gt(iw,ih),-2,min({max_image_size},ih))'"
        )

    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(video_path),
            "-vf",
            filter_chain,
            "-q:v",
            "2",
            str(output_dir / "kf_%06d.jpg"),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    extracted = sorted(output_dir.glob("kf_*.jpg"))
    if not extracted:
        raise RuntimeError("Frame extraction produced no keyframes.")

    if len(extracted) > max_keyframes:
        selected = _select_evenly_spaced(
            [{"path": path} for path in extracted],
            max_keyframes,
        )
        selected_paths = [item["path"] for item in selected]
        keep = {path.name for path in selected_paths}
        for path in extracted:
            if path.name not in keep:
                path.unlink()
        extracted = selected_paths

    metadata = probe_video(video_path)
    keyframes: list[Keyframe] = []
    for index, image_path in enumerate(extracted):
        timestamp_s = index / sample_fps
        keyframes.append(
            Keyframe(
                keyframe_id=f"kf_{index + 1:03d}",
                image_name=image_path.name,
                image_path=str(image_path),
                frame_index=index,
                timestamp_s=min(timestamp_s, metadata.duration_s),
                width=metadata.width,
                height=metadata.height,
            )
        )
    return keyframes
