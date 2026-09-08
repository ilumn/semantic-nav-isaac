from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    from tb3_semantic_map.anchor_builder import build_anchors
    from tb3_semantic_map.place_builder import build_places
    from tb3_semantic_map.relation_builder import build_relations
except ImportError as exc:
    pytest.skip(f"ROS message modules not available for semantic layer tests: {exc}", allow_module_level=True)


def _record(entity_id: str, semantic_name: str, detector_label: str, x: float, y: float):
    return SimpleNamespace(
        entity_id=entity_id,
        semantic_name=semantic_name,
        detector_label=detector_label,
        x=x,
        y=y,
        z=0.0,
        confidence=0.8,
        extent_xyz=(0.25, 0.25, 0.25),
        place_id="",
        state="active",
        aliases=[],
        provenance=["live"],
        observation_count=1,
        last_seen=1.0,
    )


def test_build_places_clusters_nearby_entities():
    records = [
        _record("person_0", "person", "person", 0.0, 0.0),
        _record("bench_0", "table", "bench", 0.6, 0.2),
        _record("stop_sign_0", "stop_sign", "stop sign", 4.0, 4.0),
    ]
    places, entity_to_place = build_places(records, stamp=SimpleNamespace(), frame_id="map", cluster_distance_m=1.0)

    assert len(places) == 2
    assert entity_to_place["person_0"] == entity_to_place["bench_0"]
    assert entity_to_place["stop_sign_0"] != entity_to_place["person_0"]


def test_build_relations_creates_near_relation_for_close_entities():
    records = [
        _record("person_0", "person", "person", 0.0, 0.0),
        _record("bench_0", "table", "bench", 0.4, 0.2),
        _record("stop_sign_0", "stop_sign", "stop sign", 3.0, 3.0),
    ]
    relations = build_relations(records, stamp=SimpleNamespace(), frame_id="map", near_distance_m=1.0)

    assert len(relations) == 1
    assert relations[0].predicate == "near"


def test_build_anchors_creates_one_anchor_per_entity():
    records = [
        _record("person_0", "person", "person", 1.0, 2.0),
        _record("bench_0", "table", "bench", 3.0, 4.0),
    ]
    anchors = build_anchors(records, stamp=SimpleNamespace(), frame_id="map")

    assert len(anchors) == 2
    assert anchors[0].target_id == "person_0"
