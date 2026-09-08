from __future__ import annotations

import json
import shutil
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .config import (
    FRONTEND_DIR,
    JOBS_DIR,
    PipelineConfig,
    configure_environment,
    default_prompt_preset,
    default_yolo_model_size,
    prompt_presets,
    resolve_prompt_request,
    resolve_yolo_model_selection,
    yolo_world_model_variants,
)
from .models import JobRecord
from .pipeline import run_pipeline


configure_environment()

app = FastAPI(title="Monocular Semantic Mapping Demo")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

_jobs: dict[str, JobRecord] = {}
_lock = threading.Lock()
_pipeline_lock = threading.Lock()


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _job_file(job_id: str) -> Path:
    return JOBS_DIR / job_id / "job.json"


def _save_job(record: JobRecord) -> None:
    _job_file(record.job_id).parent.mkdir(parents=True, exist_ok=True)
    _job_file(record.job_id).write_text(record.model_dump_json(indent=2), encoding="utf-8")


def _update_job(job_id: str, **changes) -> JobRecord:
    with _lock:
        if job_id not in _jobs:
            raise KeyError(job_id)
        record = _jobs[job_id].model_copy(update={**changes, "updated_at": _utc_now()})
        _jobs[job_id] = record
        _save_job(record)
        return record


def _load_job(job_id: str) -> JobRecord:
    with _lock:
        if job_id in _jobs:
            return _jobs[job_id]
    path = _job_file(job_id)
    if not path.exists():
        raise HTTPException(status_code=404, detail="Unknown job")
    payload = json.loads(path.read_text(encoding="utf-8"))
    record = JobRecord.model_validate(payload)
    with _lock:
        _jobs[job_id] = record
    return record


def _worker(
    job_id: str,
    video_path: Path,
    prompt_preset: str,
    scene_profile: str,
    prompt_vocabulary: list[str],
    yolo_model_size: str,
    yolo_model_name: str,
) -> None:
    job_dir = JOBS_DIR / job_id

    def progress(step: str, value: float) -> None:
        _update_job(job_id, status="running", current_step=step, progress=value)

    try:
        _update_job(
            job_id,
            status="queued",
            current_step="waiting_for_slot",
            progress=0.01,
        )
        with _pipeline_lock:
            scene_graph = run_pipeline(
                job_id=job_id,
                job_dir=job_dir,
                config=PipelineConfig(
                    prompt_vocabulary=prompt_vocabulary,
                    prompt_preset=prompt_preset,
                    yolo_model_size=yolo_model_size,
                    yolo_model_name=yolo_model_name,
                    scene_profile=scene_profile,
                ),
                progress=progress,
                video_path=video_path,
            )
            _update_job(
                job_id,
                status="completed",
                current_step="completed",
                progress=1.0,
                scene_graph_url=f"/jobs/{job_id}/outputs/scene_graph.json",
                artifacts=scene_graph.artifacts,
            )
    except Exception as exc:  # noqa: BLE001
        _update_job(
            job_id,
            status="failed",
            current_step="failed",
            error=str(exc),
        )


@app.get("/api/health")
def healthcheck() -> dict[str, str]:
    return {"status": "ok"}


@app.get("/api/config")
def app_config() -> dict:
    presets = prompt_presets()
    return {
        "default_prompt_preset": default_prompt_preset(),
        "default_yolo_model_size": default_yolo_model_size(),
        "prompt_presets": {
            key: {
                "label": value.get("label", key.title()),
                "description": value.get("description", ""),
                "scene_profile": value.get("scene_profile", "general"),
                "prompts": value.get("prompts", []),
            }
            for key, value in presets.items()
        },
        "yolo_world_models": yolo_world_model_variants(),
    }


@app.post("/api/jobs", response_model=JobRecord)
async def create_job(
    file: UploadFile = File(...),
    preset: str = Form(""),
    model_size: str = Form(""),
    prompts: str = Form(""),
) -> JobRecord:
    if not file.filename:
        raise HTTPException(status_code=400, detail="Missing filename")

    job_id = uuid.uuid4().hex[:12]
    job_dir = JOBS_DIR / job_id
    input_dir = job_dir / "input"
    input_dir.mkdir(parents=True, exist_ok=True)
    input_path = input_dir / file.filename

    with input_path.open("wb") as handle:
        shutil.copyfileobj(file.file, handle)

    extra_prompts = [item.strip() for item in prompts.replace("\n", ",").split(",") if item.strip()]
    prompt_preset, scene_profile, prompt_vocabulary = resolve_prompt_request(file.filename, preset, extra_prompts)
    selected_model_size, selected_model_name = resolve_yolo_model_selection(model_size)

    record = JobRecord(
        job_id=job_id,
        status="queued",
        current_step="queued",
        created_at=_utc_now(),
        updated_at=_utc_now(),
        input_video=str(input_path),
        input_video_url=f"/jobs/{job_id}/input/{file.filename}",
        progress=0.0,
        prompt_preset=prompt_preset,
        model_size=selected_model_size,
        model_name=selected_model_name,
        scene_profile=scene_profile,
        prompt_count=len(prompt_vocabulary),
        prompt_preview=prompt_vocabulary[:8],
    )
    with _lock:
        _jobs[job_id] = record
    _save_job(record)

    thread = threading.Thread(
        target=_worker,
        args=(
            job_id,
            input_path,
            prompt_preset,
            scene_profile,
            prompt_vocabulary,
            selected_model_size,
            selected_model_name,
        ),
        daemon=True,
    )
    thread.start()
    return record


@app.get("/api/jobs/{job_id}", response_model=JobRecord)
def get_job(job_id: str) -> JobRecord:
    return _load_job(job_id)


@app.get("/api/jobs/{job_id}/scene-graph")
def get_scene_graph(job_id: str):
    scene_graph_path = JOBS_DIR / job_id / "outputs" / "scene_graph.json"
    if not scene_graph_path.exists():
        raise HTTPException(status_code=404, detail="Scene graph not available")
    return FileResponse(scene_graph_path)


@app.get("/")
def frontend_index() -> FileResponse:
    return FileResponse(FRONTEND_DIR / "index.html")


app.mount("/jobs", StaticFiles(directory=JOBS_DIR), name="jobs")
app.mount("/", StaticFiles(directory=FRONTEND_DIR, html=True), name="frontend")


def main() -> None:
    import uvicorn

    uvicorn.run("semantic_nav_memory.server:app", host="127.0.0.1", port=8000, reload=False)
