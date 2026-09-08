from __future__ import annotations

import json
import os
import shutil
import subprocess
import uuid
from dataclasses import dataclass, field
from pathlib import Path

from .alignment_core import estimate_alignment_2d
from .job_core import RefinerJobRecord, utc_now


@dataclass(slots=True)
class WorkerConfig:
    python_executable: str = "python3.12"
    frame_sample_fps: float = 1.0
    max_keyframes: int = 128
    max_image_size: int = 1440
    yolo_model_size: str = ""
    yolo_model_name: str = ""
    yolo_confidence: float = 0.18
    prompt_preset: str = ""
    scene_profile: str = "general"
    prompt_vocabulary: list[str] = field(default_factory=list)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[6]


def _semantic_nav_memory_candidates() -> list[Path]:
    repo_root = _repo_root()
    return [
        repo_root / "dev" / "turtlebot3" / "external" / "semantic-nav-memory",
        repo_root / "semantic-nav-memory",
    ]


def _semantic_nav_memory_exists(path: Path) -> bool:
    return (path / "semantic_nav_memory" / "cli.py").exists()


def _semantic_nav_memory_root() -> Path:
    for candidate in _semantic_nav_memory_candidates():
        if _semantic_nav_memory_exists(candidate):
            return candidate
    return _semantic_nav_memory_candidates()[0]


def _semantic_nav_memory_missing_message() -> str:
    searched = ", ".join(str(path) for path in _semantic_nav_memory_candidates())
    return f"semantic-nav-memory repo not found; searched: {searched}"


def _local_worker_python(configured_python: str) -> str:
    sem_root = _semantic_nav_memory_root()
    repo_root = _repo_root()
    if Path(configured_python).is_absolute():
        return configured_python

    candidates: list[Path] = []
    configured_venv = os.environ.get("ISAAC_SEMANTIC_VENV", "").strip()
    if configured_venv:
        candidates.append(Path(configured_venv).expanduser() / "bin" / "python")
    candidates.extend((
        repo_root / ".venv" / "bin" / "python",
        sem_root / ".venv-cpu" / "bin" / "python",
        sem_root / ".venv" / "bin" / "python",
    ))
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return configured_python


def snapshot_latest_bundle(bundle_root: str) -> tuple[str, Path, dict]:
    latest_dir = Path(bundle_root) / "latest"
    manifest_path = latest_dir / "manifest.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"Bundle manifest not found: {manifest_path}")

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    job_id = uuid.uuid4().hex[:12]
    jobs_dir = Path(bundle_root) / "jobs"
    job_dir = jobs_dir / job_id
    snapshot_dir = job_dir / "bundle"
    shutil.copytree(latest_dir, snapshot_dir)
    return job_id, job_dir, manifest


def _sorted_frames(manifest: dict) -> list[dict]:
    return sorted(manifest.get("frames", []), key=lambda item: item.get("stamp_sec", 0.0))


def _bundle_fps(manifest: dict) -> float:
    frames = _sorted_frames(manifest)
    if len(frames) < 2:
        return 1.0
    deltas = [
        max(1e-3, frames[index + 1]["stamp_sec"] - frames[index]["stamp_sec"])
        for index in range(len(frames) - 1)
    ]
    avg_delta = sum(deltas) / len(deltas)
    return max(0.5, 1.0 / avg_delta)


def _progress_update(job: RefinerJobRecord, job_path: Path, step: str, progress: float) -> None:
    job.current_step = step
    job.progress = progress
    job.updated_at = utc_now()
    job.save(job_path)


def _bundle_camera_intrinsics(manifest: dict) -> dict[str, float | int] | None:
    for frame in _sorted_frames(manifest):
        fx = frame.get("fx")
        fy = frame.get("fy")
        cx = frame.get("cx")
        cy = frame.get("cy")
        width = frame.get("width")
        height = frame.get("height")
        if None in (fx, fy, cx, cy):
            continue
        payload: dict[str, float | int] = {
            "fx": float(fx),
            "fy": float(fy),
            "cx": float(cx),
            "cy": float(cy),
        }
        if width is not None:
            payload["width"] = int(width)
        if height is not None:
            payload["height"] = int(height)
        return payload
    return None


def _build_worker_command(
    job_id: str,
    image_dir: Path,
    job_dir: Path,
    worker_config: WorkerConfig,
    *,
    frame_manifest_path: Path | None = None,
    camera_intrinsics: dict[str, float | int] | None = None,
) -> list[str]:
    command = [
        _local_worker_python(worker_config.python_executable),
        "-m",
        "semantic_nav_memory.cli",
        "--job-id",
        job_id,
        "--image-dir",
        str(image_dir),
        "--job-dir",
        str(job_dir),
        "--frame-sample-fps",
        str(worker_config.frame_sample_fps),
        "--max-keyframes",
        str(worker_config.max_keyframes),
        "--max-image-size",
        str(worker_config.max_image_size),
        "--yolo-confidence",
        str(worker_config.yolo_confidence),
        "--scene-profile",
        worker_config.scene_profile,
    ]
    if frame_manifest_path is not None:
        command.extend(["--frame-manifest", str(frame_manifest_path)])
    if camera_intrinsics is not None:
        if "fx" in camera_intrinsics:
            command.extend(["--camera-fx", str(camera_intrinsics["fx"])])
        if "fy" in camera_intrinsics:
            command.extend(["--camera-fy", str(camera_intrinsics["fy"])])
        if "cx" in camera_intrinsics:
            command.extend(["--camera-cx", str(camera_intrinsics["cx"])])
        if "cy" in camera_intrinsics:
            command.extend(["--camera-cy", str(camera_intrinsics["cy"])])
        if "width" in camera_intrinsics:
            command.extend(["--camera-width", str(camera_intrinsics["width"])])
        if "height" in camera_intrinsics:
            command.extend(["--camera-height", str(camera_intrinsics["height"])])
    if worker_config.yolo_model_size:
        command.extend(["--yolo-model-size", worker_config.yolo_model_size])
    if worker_config.yolo_model_name:
        command.extend(["--yolo-model-name", worker_config.yolo_model_name])
    if worker_config.prompt_preset:
        command.extend(["--prompt-preset", worker_config.prompt_preset])
    for prompt in worker_config.prompt_vocabulary:
        if prompt:
            command.extend(["--prompt", prompt])
    return command


def _run_worker_subprocess(
    command: list[str],
    worker_timeout_sec: float,
) -> subprocess.CompletedProcess[str]:
    sem_root = _semantic_nav_memory_root()
    if not _semantic_nav_memory_exists(sem_root):
        raise FileNotFoundError(_semantic_nav_memory_missing_message())

    env = os.environ.copy()
    pythonpath_prefix = str(sem_root)
    if env.get("PYTHONPATH"):
        env["PYTHONPATH"] = f"{pythonpath_prefix}{os.pathsep}{env['PYTHONPATH']}"
    else:
        env["PYTHONPATH"] = pythonpath_prefix

    local_bin = sem_root / ".local-bin"
    if local_bin.exists():
        path_prefix = str(local_bin)
        if env.get("PATH"):
            env["PATH"] = f"{path_prefix}{os.pathsep}{env['PATH']}"
        else:
            env["PATH"] = path_prefix

    return subprocess.run(
        command,
        cwd=str(sem_root),
        env=env,
        capture_output=True,
        text=True,
        timeout=max(1.0, worker_timeout_sec),
        check=False,
    )


def _write_pose_correspondences(job_dir: Path, manifest: dict, scene_graph: dict) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    frames = _sorted_frames(manifest)
    cameras = sorted(
        scene_graph.get("reconstruction", {}).get("cameras", []),
        key=lambda item: item.get("keyframe_id", ""),
    )

    correspondences: list[dict] = []
    source_points: list[tuple[float, float]] = []
    target_points: list[tuple[float, float]] = []

    for camera, frame in zip(cameras, frames):
        robot_x = frame.get("robot_x")
        robot_y = frame.get("robot_y")
        center = camera.get("center", [])
        if robot_x is None or robot_y is None or len(center) < 2:
            continue
        source_xy = (float(center[0]), float(center[1]))
        target_xy = (float(robot_x), float(robot_y))
        source_points.append(source_xy)
        target_points.append(target_xy)
        correspondences.append(
            {
                "keyframe_id": camera.get("keyframe_id", ""),
                "image_name": camera.get("image_name", ""),
                "camera_center_xy": [source_xy[0], source_xy[1]],
                "robot_map_xy": [target_xy[0], target_xy[1]],
                "stamp_sec": frame.get("stamp_sec"),
                "frame_index": frame.get("frame_index"),
            }
        )

    (job_dir / "pose_correspondences.json").write_text(
        json.dumps(correspondences, indent=2),
        encoding="utf-8",
    )
    return source_points, target_points


def _estimate_alignment(job_dir: Path, manifest: dict, scene_graph: dict) -> dict:
    source_points, target_points = _write_pose_correspondences(job_dir, manifest, scene_graph)

    if len(source_points) < 2:
        alignment = {
            "status": "insufficient_correspondences",
            "accepted": False,
            "confidence": 0.0,
            "correspondences": len(source_points),
        }
    else:
        estimate = estimate_alignment_2d(source_points, target_points)
        alignment = {
            "status": "ok" if estimate.accepted else "rejected",
            "accepted": estimate.accepted,
            "confidence": estimate.confidence,
            "correspondences": estimate.correspondences,
            "scale": estimate.transform.scale,
            "yaw_rad": estimate.transform.yaw_rad,
            "tx": estimate.transform.tx,
            "ty": estimate.transform.ty,
            "mean_error_m": estimate.mean_error,
            "rms_error_m": estimate.rms_error,
            "max_error_m": estimate.max_error,
            "source_span_m": estimate.source_span,
            "target_span_m": estimate.target_span,
            "planar_consistency": max(0.0, min(1.0, 1.0 - estimate.rms_error / max(estimate.target_span, 1e-6))),
        }
    (job_dir / "alignment.json").write_text(json.dumps(alignment, indent=2), encoding="utf-8")
    return alignment


def load_latest_job_record(bundle_root: str) -> RefinerJobRecord | None:
    jobs_dir = Path(bundle_root) / "jobs"
    if not jobs_dir.exists():
        return None
    candidates = sorted(
        [path / "job.json" for path in jobs_dir.iterdir() if path.is_dir() and (path / "job.json").exists()],
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        return None
    payload = json.loads(candidates[0].read_text(encoding="utf-8"))
    return RefinerJobRecord(**payload)


def run_semantic_nav_memory_job(
    bundle_root: str,
    worker_timeout_sec: float,
    worker_config: WorkerConfig | None = None,
) -> RefinerJobRecord:
    worker_config = worker_config or WorkerConfig()
    job_id, job_dir, manifest = snapshot_latest_bundle(bundle_root)
    job = RefinerJobRecord(
        job_id=job_id,
        status="running",
        created_at=utc_now(),
        updated_at=utc_now(),
        progress=0.01,
        current_step="snapshot_ready",
        buffered_frame_count=len(manifest.get("frames", [])),
        bundle_dir=str(job_dir / "bundle"),
        work_dir=str(job_dir),
    )
    job_path = job_dir / "job.json"
    job.save(job_path)

    try:
        fps = max(0.5, worker_config.frame_sample_fps or _bundle_fps(manifest))
        frame_dir = job_dir / "bundle" / "frames"
        frame_manifest_path = job_dir / "bundle" / "manifest.json"
        if not frame_dir.exists():
            raise RuntimeError(f"Sampled frame directory not found: {frame_dir}")
        camera_intrinsics = _bundle_camera_intrinsics(manifest)

        _progress_update(job, job_path, "preparing_image_sequence", 0.05)

        effective_config = WorkerConfig(
            python_executable=worker_config.python_executable,
            frame_sample_fps=fps,
            max_keyframes=max(8, worker_config.max_keyframes),
            max_image_size=max(256, worker_config.max_image_size),
            yolo_model_size=worker_config.yolo_model_size,
            yolo_model_name=worker_config.yolo_model_name,
            yolo_confidence=worker_config.yolo_confidence,
            prompt_preset=worker_config.prompt_preset,
            scene_profile=worker_config.scene_profile,
            prompt_vocabulary=list(worker_config.prompt_vocabulary),
        )
        command = _build_worker_command(
            job_id=job_id,
            image_dir=frame_dir,
            job_dir=job_dir,
            worker_config=effective_config,
            frame_manifest_path=frame_manifest_path,
            camera_intrinsics=camera_intrinsics,
        )
        job.worker_command = list(command)
        _progress_update(job, job_path, "running_worker", 0.08)

        completed = _run_worker_subprocess(command, worker_timeout_sec)
        job.worker_stdout = completed.stdout[-12000:]
        job.worker_stderr = completed.stderr[-12000:]
        if completed.returncode != 0:
            job.status = "failed"
            job.current_step = "worker_failed"
            job.error = f"worker exited with code {completed.returncode}"
            job.updated_at = utc_now()
            job.save(job_path)
            return job

        scene_graph_path = job_dir / "outputs" / "scene_graph.json"
        if not scene_graph_path.exists():
            job.status = "failed"
            job.current_step = "missing_scene_graph"
            job.error = f"Missing scene graph output: {scene_graph_path}"
            job.updated_at = utc_now()
            job.save(job_path)
            return job

        scene_graph = json.loads(scene_graph_path.read_text(encoding="utf-8"))
        job.output_scene_graph_path = str(scene_graph_path)
        job.alignment = _estimate_alignment(job_dir, manifest, scene_graph)
        job.output_alignment_path = str(job_dir / "alignment.json")
        job.accepted_refinement = bool(job.alignment.get("accepted", False))
        job.status = "completed"
        job.current_step = "completed"
        job.progress = 1.0
        job.updated_at = utc_now()
        job.save(job_path)
        return job
    except subprocess.TimeoutExpired as exc:
        job.status = "failed"
        job.current_step = "timeout"
        job.error = f"worker timed out after {worker_timeout_sec:.1f}s"
        job.worker_stdout = (exc.stdout or "")[-12000:]
        job.worker_stderr = (exc.stderr or "")[-12000:]
        job.updated_at = utc_now()
        job.save(job_path)
        return job
    except Exception as exc:  # noqa: BLE001
        job.status = "failed"
        job.current_step = "failed"
        job.error = str(exc)
        job.updated_at = utc_now()
        job.save(job_path)
        return job


def cleanup_old_jobs(bundle_root: str, max_completed_jobs: int) -> None:
    jobs_dir = Path(bundle_root) / "jobs"
    if not jobs_dir.exists():
        return
    job_dirs = [path for path in jobs_dir.iterdir() if path.is_dir()]
    latest_accepted: Path | None = None
    accepted_mtime = -1.0
    for path in job_dirs:
        job_path = path / "job.json"
        if not job_path.exists():
            continue
        try:
            payload = json.loads(job_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if payload.get("status") == "completed" and payload.get("accepted_refinement", False):
            mtime = job_path.stat().st_mtime
            if mtime > accepted_mtime:
                accepted_mtime = mtime
                latest_accepted = path

    job_dirs.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    stale_jobs = list(job_dirs[max(0, max_completed_jobs):])
    if latest_accepted is not None and latest_accepted in stale_jobs:
        stale_jobs.remove(latest_accepted)
    for stale in stale_jobs:
        shutil.rmtree(stale, ignore_errors=True)
