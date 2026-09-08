from __future__ import annotations

from pathlib import Path

from tb3_semantic_refiner.worker_core import (
    WorkerConfig,
    _build_worker_command,
    _local_worker_python,
    _semantic_nav_memory_root,
)


def test_semantic_nav_memory_root_prefers_vendored_source() -> None:
    root = _semantic_nav_memory_root()

    assert root.as_posix().endswith("dev/turtlebot3/external/semantic-nav-memory")
    assert (root / "semantic_nav_memory" / "cli.py").exists()


def test_local_worker_python_prefers_configured_isaac_venv(tmp_path, monkeypatch) -> None:
    python = tmp_path / "bin" / "python"
    python.parent.mkdir()
    python.touch()
    monkeypatch.setenv("ISAAC_SEMANTIC_VENV", str(tmp_path))

    assert _local_worker_python("python3") == str(python)


def test_build_worker_command_uses_image_sequence_inputs() -> None:
    command = _build_worker_command(
        job_id="job123",
        image_dir=Path("/tmp/bundle/frames"),
        job_dir=Path("/tmp/job"),
        worker_config=WorkerConfig(prompt_vocabulary=["desk", "person"], prompt_preset="indoor"),
        frame_manifest_path=Path("/tmp/bundle/manifest.json"),
        camera_intrinsics={
            "fx": 320.0,
            "fy": 321.0,
            "cx": 160.0,
            "cy": 120.0,
            "width": 320,
            "height": 240,
        },
    )

    assert "--image-dir" in command
    assert "/tmp/bundle/frames" in command
    assert "--frame-manifest" in command
    assert "/tmp/bundle/manifest.json" in command
    assert "--video-path" not in command
    assert "--camera-fx" in command
    assert "--camera-height" in command
    assert command.count("--prompt") == 2


def test_build_worker_command_skips_optional_intrinsics_when_missing() -> None:
    command = _build_worker_command(
        job_id="job123",
        image_dir=Path("/tmp/bundle/frames"),
        job_dir=Path("/tmp/job"),
        worker_config=WorkerConfig(),
    )

    assert "--camera-fx" not in command
    assert "--camera-fy" not in command
    assert "--frame-manifest" not in command
