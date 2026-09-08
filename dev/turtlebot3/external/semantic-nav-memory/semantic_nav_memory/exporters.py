from __future__ import annotations

import json
from pathlib import Path

from pydantic import BaseModel


def write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(payload, BaseModel):
        path.write_text(payload.model_dump_json(indent=2), encoding="utf-8")
        return
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
