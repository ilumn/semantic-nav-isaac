from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from semantic_nav_memory.config import PipelineConfig
from semantic_nav_memory.geometry_colmap import AttemptConfig, COLMAPGeometryBackend
from semantic_nav_memory.models import Keyframe


class _FakeImage:
    def __init__(self, name: str) -> None:
        self.name = name


class _FakeReconstruction:
    def __init__(self) -> None:
        self._image_names = {
            1: "kf_000001.jpg",
            2: "kf_000002.jpg",
        }

    def num_reg_images(self) -> int:
        return len(self._image_names)

    def num_points3D(self) -> int:
        return 42

    def reg_image_ids(self) -> list[int]:
        return list(self._image_names.keys())

    def image(self, image_id: int) -> _FakeImage:
        return _FakeImage(self._image_names[image_id])


class GeometryColmapTests(unittest.TestCase):
    def test_run_attempt_invokes_incremental_mapping_for_sequential_matcher(self) -> None:
        backend = COLMAPGeometryBackend(PipelineConfig())
        keyframes = [
            Keyframe(
                keyframe_id="kf_001",
                image_name="kf_000001.jpg",
                image_path="/tmp/kf_000001.jpg",
                frame_index=0,
                timestamp_s=0.0,
                width=640,
                height=480,
            ),
            Keyframe(
                keyframe_id="kf_002",
                image_name="kf_000002.jpg",
                image_path="/tmp/kf_000002.jpg",
                frame_index=1,
                timestamp_s=1.0,
                width=640,
                height=480,
            ),
        ]

        with tempfile.TemporaryDirectory() as tmp:
            workspace_dir = Path(tmp)
            (workspace_dir / "images").mkdir()

            with (
                patch("semantic_nav_memory.geometry_colmap.pycolmap.extract_features"),
                patch("semantic_nav_memory.geometry_colmap.pycolmap.match_sequential"),
                patch(
                    "semantic_nav_memory.geometry_colmap.pycolmap.incremental_mapping",
                    return_value={1: _FakeReconstruction()},
                ) as mock_incremental_mapping,
            ):
                attempt = backend._run_attempt(
                    keyframes=keyframes,
                    workspace_dir=workspace_dir,
                    attempt_config=AttemptConfig(
                        label="sequential_test",
                        matcher="sequential",
                        camera_model="SIMPLE_RADIAL",
                    ),
                )

        self.assertEqual(attempt.summary, "sequential_test:2/2:pts=42")
        self.assertEqual(attempt.camera_model, "SIMPLE_RADIAL")
        self.assertIsNone(attempt.error)
        self.assertTrue(mock_incremental_mapping.called)


if __name__ == "__main__":
    unittest.main()
