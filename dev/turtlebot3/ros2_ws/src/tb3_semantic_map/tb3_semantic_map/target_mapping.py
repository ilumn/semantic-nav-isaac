from __future__ import annotations

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def default_targets_file() -> str:
    pkg_share = get_package_share_directory("tb3_frontier_exploration")
    return os.path.join(pkg_share, "config", "semantic_targets.yaml")


def load_target_mapping(yaml_path: str | Path) -> tuple[dict[str, str], dict[str, str]]:
    path = Path(yaml_path)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    targets = data.get("semantic_targets", [])
    sem2det: dict[str, str] = {}
    det2sem: dict[str, str] = {}
    for entry in targets:
        if not entry.get("enabled", True):
            continue
        semantic_name = str(entry["semantic_name"])
        detector_label = str(entry["detector_label"])
        sem2det[semantic_name] = detector_label
        det2sem[detector_label] = semantic_name
    return sem2det, det2sem


def semantic_name_for_label(detector_label: str, det2sem: dict[str, str]) -> str:
    if detector_label in det2sem:
        return det2sem[detector_label]
    return detector_label.strip().lower().replace(" ", "_")
