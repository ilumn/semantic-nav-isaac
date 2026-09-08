from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from semantic_nav_memory.video_io import extract_keyframes_from_sequence, probe_image_sequence


def _write_image(path: Path, width: int, height: int, value: int) -> None:
    image = np.full((height, width, 3), value, dtype=np.uint8)
    assert cv2.imwrite(str(path), image)


def test_extract_keyframes_from_sequence_uses_manifest_timestamps_and_endpoints(tmp_path: Path) -> None:
    input_dir = tmp_path / "frames"
    input_dir.mkdir()
    manifest_path = tmp_path / "manifest.json"

    for index in range(3):
        _write_image(input_dir / f"frame_{index:06d}.jpg", width=80, height=40, value=40 * (index + 1))

    manifest_path.write_text(
        json.dumps(
            {
                "frames": [
                    {
                        "image_path": str(input_dir / "frame_000000.jpg"),
                        "frame_index": 4,
                        "stamp_sec": 10.0,
                    },
                    {
                        "image_path": str(input_dir / "frame_000001.jpg"),
                        "frame_index": 5,
                        "stamp_sec": 10.4,
                    },
                    {
                        "image_path": str(input_dir / "frame_000002.jpg"),
                        "frame_index": 6,
                        "stamp_sec": 11.0,
                    },
                ]
            }
        ),
        encoding="utf-8",
    )

    keyframes = extract_keyframes_from_sequence(
        image_dir=input_dir,
        output_dir=tmp_path / "keyframes",
        max_keyframes=2,
        max_image_size=40,
        frame_manifest_path=manifest_path,
    )

    assert [keyframe.frame_index for keyframe in keyframes] == [4, 6]
    assert [round(keyframe.timestamp_s, 3) for keyframe in keyframes] == [0.0, 1.0]
    assert [(keyframe.width, keyframe.height) for keyframe in keyframes] == [(40, 20), (40, 20)]
    assert all(Path(keyframe.image_path).exists() for keyframe in keyframes)


def test_probe_image_sequence_uses_manifest_timing(tmp_path: Path) -> None:
    input_dir = tmp_path / "frames"
    input_dir.mkdir()
    manifest_path = tmp_path / "manifest.json"

    for index in range(3):
        _write_image(input_dir / f"frame_{index:06d}.jpg", width=64, height=48, value=30 * (index + 1))

    manifest_path.write_text(
        json.dumps(
            {
                "frames": [
                    {
                        "image_path": str(input_dir / "frame_000000.jpg"),
                        "frame_index": 0,
                        "stamp_sec": 2.0,
                    },
                    {
                        "image_path": str(input_dir / "frame_000001.jpg"),
                        "frame_index": 1,
                        "stamp_sec": 2.5,
                    },
                    {
                        "image_path": str(input_dir / "frame_000002.jpg"),
                        "frame_index": 2,
                        "stamp_sec": 3.0,
                    },
                ]
            }
        ),
        encoding="utf-8",
    )

    metadata = probe_image_sequence(input_dir, frame_manifest_path=manifest_path)

    assert metadata.width == 64
    assert metadata.height == 48
    assert metadata.frame_count == 3
    assert metadata.duration_s == 1.0
    assert metadata.fps == 2.0
