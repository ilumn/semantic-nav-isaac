from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tb3_query.query_core import (
    MemoryAnchor,
    MemoryObject,
    MemoryRelation,
    attach_anchor,
    parse_command,
    select_best_anchor,
    select_relational_target,
    select_target,
)


def test_parse_command_supports_target_only_and_stop_sign_alias():
    parsed = parse_command("go to the stop sign", {"person", "table", "stop_sign"})

    assert parsed is not None
    assert parsed.target_semantic_name == "stop_sign"
    assert parsed.has_relation is False


def test_parse_command_normalizes_relational_aliases_to_near():
    parsed = parse_command("go to the person left of the table", {"person", "table"})

    assert parsed is not None
    assert parsed.target_semantic_name == "person"
    assert parsed.reference_semantic_name == "table"
    assert parsed.relation_predicate == "near"
    assert parsed.relation_phrase == "left of"


def test_select_target_picks_nearest_matching_detector_label():
    objects = [
        MemoryObject(object_id="person_0", detector_label="person", semantic_name="person", x=2.0, y=0.0, confidence=0.7),
        MemoryObject(object_id="person_1", detector_label="person", semantic_name="person", x=1.0, y=0.0, confidence=0.9),
        MemoryObject(object_id="bench_0", detector_label="bench", semantic_name="table", x=0.5, y=0.0, confidence=0.8),
    ]

    result = select_target(objects, "person", "person", "go to the person")

    assert result.success is True
    assert result.object_id == "person_1"
    assert result.semantic_name == "person"


def test_select_best_anchor_prefers_highest_confidence_for_target():
    anchors = [
        MemoryAnchor("anchor_0", "inspection", "person_0", 1.0, 2.0, 0.1, 0.4),
        MemoryAnchor("anchor_1", "inspection", "person_0", 2.0, 3.0, 0.2, 0.8),
        MemoryAnchor("anchor_2", "inspection", "person_1", 3.0, 4.0, 0.3, 0.9),
    ]

    best = select_best_anchor(anchors, "person_0")

    assert best is not None
    assert best.anchor_id == "anchor_1"
    assert best.confidence == 0.8


def test_attach_anchor_augments_successful_query_result():
    result = select_target(
        [MemoryObject(object_id="person_0", detector_label="person", semantic_name="person", x=1.0, y=0.0, confidence=0.9)],
        "person",
        "person",
        "go to the person",
    )
    anchor = MemoryAnchor("anchor_1", "inspection", "person_0", 0.5, 0.1, 1.2, 0.85)

    updated = attach_anchor(result, anchor)

    assert updated.has_anchor is True
    assert updated.anchor_id == "anchor_1"
    assert updated.anchor_type == "inspection"
    assert updated.anchor_x == 0.5
    assert updated.anchor_yaw == 1.2
    assert updated.anchor_confidence == 0.85


def test_select_relational_target_prefers_strongest_relation_over_nearest_target():
    objects = [
        MemoryObject("person_0", "person", 1.0, 0.0, 0.7, "person"),
        MemoryObject("person_1", "person", 3.0, 0.0, 0.8, "person"),
        MemoryObject("bench_0", "bench", 2.0, 0.0, 0.9, "table"),
    ]
    relations = [
        MemoryRelation("person_0", "near", "bench_0", 0.35),
        MemoryRelation("person_1", "near", "bench_0", 0.82),
    ]

    result = select_relational_target(
        objects,
        relations,
        "person",
        "person",
        "table",
        "bench",
        "near",
        "go to the person near the table",
    )

    assert result.success is True
    assert result.object_id == "person_1"
    assert "near bench_0" in result.status_message


def test_select_relational_target_breaks_ties_by_target_distance():
    objects = [
        MemoryObject("person_0", "person", 1.0, 0.0, 0.7, "person"),
        MemoryObject("person_1", "person", 2.0, 0.0, 0.95, "person"),
        MemoryObject("bench_0", "bench", 3.0, 0.0, 0.9, "table"),
    ]
    relations = [
        MemoryRelation("person_0", "near", "bench_0", 0.8),
        MemoryRelation("person_1", "near", "bench_0", 0.8),
    ]

    result = select_relational_target(
        objects,
        relations,
        "person",
        "person",
        "table",
        "bench",
        "near",
        "go to the person near the table",
    )

    assert result.success is True
    assert result.object_id == "person_0"


def test_select_relational_target_treats_near_as_symmetric():
    objects = [
        MemoryObject("person_0", "person", 1.0, 0.0, 0.7, "person"),
        MemoryObject("bench_0", "bench", 2.0, 0.0, 0.9, "table"),
    ]
    relations = [
        MemoryRelation("bench_0", "near", "person_0", 0.88),
    ]

    result = select_relational_target(
        objects,
        relations,
        "person",
        "person",
        "table",
        "bench",
        "near",
        "go to the person right of the table",
    )

    assert result.success is True
    assert result.object_id == "person_0"
