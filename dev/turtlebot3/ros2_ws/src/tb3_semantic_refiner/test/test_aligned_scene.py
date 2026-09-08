from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tb3_semantic_refiner.aligned_scene import load_aligned_scene


def test_load_aligned_scene_transforms_scene_graph_outputs(tmp_path):
    job_dir = tmp_path / "job_001"
    outputs_dir = job_dir / "outputs"
    outputs_dir.mkdir(parents=True)

    alignment = {
        "status": "ok",
        "accepted": True,
        "scale": 2.0,
        "yaw_rad": 0.0,
        "tx": 1.0,
        "ty": -1.0,
    }
    scene_graph = {
        "reconstruction": {
            "sparse_points": [
                {
                    "point_id": 5,
                    "xyz": [1.0, 1.5, 0.25],
                    "color": [10, 20, 30],
                    "error": 0.4,
                }
            ],
            "support_points": [
                {
                    "point_id": 8,
                    "xyz": [0.5, 0.0, 0.75],
                    "color": [200, 210, 220],
                    "error": 0.1,
                }
            ],
        },
        "entities": [
            {
                "entity_id": "entity_1",
                "canonical_label": "person",
                "place_id": "place_1",
                "state": "confirmed",
                "aliases": ["human"],
                "world_pose_estimate": [1.0, 2.0, 0.5],
                "extent_3d": [0.4, 0.5, 0.6],
                "confidence": 0.8,
                "observation_count": 3,
            }
        ],
        "places": [
            {
                "place_id": "place_1",
                "place_type_distribution": {"work_area": 1.0},
                "anchor_pose": [2.0, 1.0, 0.0],
                "extent_3d": [1.0, 2.0, 0.5],
                "member_entities": ["entity_1"],
                "confidence": 0.7,
            }
        ],
        "relations": [
            {
                "subject": "entity_1",
                "predicate": "in_place",
                "object": "place_1",
                "confidence": 0.9,
            }
        ],
        "task_anchors": [
            {
                "anchor_id": "anchor_1",
                "anchor_type": "inspection_anchor",
                "target_id": "entity_1",
                "pose": [0.0, 2.0, 0.5],
                "confidence": 0.6,
            }
        ],
        "diagnostics": {"entities": 1},
    }

    (job_dir / "alignment.json").write_text(json.dumps(alignment), encoding="utf-8")
    (outputs_dir / "scene_graph.json").write_text(json.dumps(scene_graph), encoding="utf-8")

    aligned = load_aligned_scene(job_dir)

    assert aligned is not None
    assert aligned.accepted is True
    assert len(aligned.entities) == 1
    assert aligned.entities[0].x == 3.0
    assert aligned.entities[0].y == 3.0
    assert aligned.entities[0].z == 1.0
    assert aligned.entities[0].extent_xyz == (0.8, 1.0, 1.2)
    assert len(aligned.places) == 1
    assert aligned.places[0].x == 5.0
    assert aligned.places[0].y == 1.0
    assert len(aligned.anchors) == 1
    assert abs(aligned.anchors[0].yaw_rad - 0.0) < 1e-6
    assert len(aligned.sparse_points) == 1
    assert aligned.sparse_points[0].x == 3.0
    assert aligned.sparse_points[0].y == 2.0
    assert aligned.sparse_points[0].z == 0.5
    assert aligned.sparse_points[0].color_rgb == (10, 20, 30)
    assert len(aligned.support_points) == 1
    assert aligned.support_points[0].x == 2.0
    assert aligned.support_points[0].y == -1.0
    assert aligned.support_points[0].z == 1.5


def test_load_aligned_scene_rejects_unaccepted_alignment_by_default(tmp_path):
    job_dir = tmp_path / "job_rejected"
    outputs_dir = job_dir / "outputs"
    outputs_dir.mkdir(parents=True)

    (job_dir / "alignment.json").write_text(
        json.dumps({"status": "weak", "accepted": False}),
        encoding="utf-8",
    )
    (outputs_dir / "scene_graph.json").write_text(
        json.dumps({"entities": [{"entity_id": "person_1", "canonical_label": "person"}]}),
        encoding="utf-8",
    )

    assert load_aligned_scene(job_dir) is None

    rejected = load_aligned_scene(job_dir, require_accepted=False)
    assert rejected is not None
    assert rejected.accepted is False
