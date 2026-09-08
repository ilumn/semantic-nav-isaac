from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any
import json
import time


@dataclass(slots=True)
class RefinerJobRecord:
    job_id: str
    status: str
    created_at: float
    updated_at: float
    progress: float = 0.0
    current_step: str = "queued"
    buffered_frame_count: int = 0
    bundle_dir: str = ""
    work_dir: str = ""
    error: str = ""
    alignment: dict[str, Any] = field(default_factory=dict)
    output_scene_graph_path: str = ""
    output_alignment_path: str = ""
    accepted_refinement: bool = False
    worker_command: list[str] = field(default_factory=list)
    worker_stdout: str = ""
    worker_stderr: str = ""

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")


def utc_now() -> float:
    return time.time()
