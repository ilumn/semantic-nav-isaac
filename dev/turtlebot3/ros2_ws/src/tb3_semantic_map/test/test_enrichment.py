from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tb3_semantic_map.enrichment import (
    enrich_state_with_refinement,
    match_refined_entities_to_live,
)


def _pose(x: float, y: float):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _entity(entity_id: str, label: str, x: float, y: float, confidence: float = 0.8):
    return SimpleNamespace(
        entity_id=entity_id,
        semantic_name=label,
        detector_label=label,
        pose=_pose(x, y),
        aliases=[],
        provenance=["live"] if entity_id.startswith("live") else ["semantic_nav_memory"],
        confidence=confidence,
    )


def _place(place_id: str, members: list[str], x: float, y: float, confidence: float = 0.7):
    return SimpleNamespace(
        place_id=place_id,
        place_type="work_area",
        member_entity_ids=list(members),
        anchor_pose=_pose(x, y),
        extent=SimpleNamespace(x=1.0, y=1.0, z=0.1),
        confidence=confidence,
    )


def _relation(subject_id: str, predicate: str, object_id: str, confidence: float = 0.8):
    return SimpleNamespace(
        subject_id=subject_id,
        predicate=predicate,
        object_id=object_id,
        confidence=confidence,
    )


def _anchor(anchor_id: str, target_id: str, x: float, y: float, confidence: float = 0.8):
    return SimpleNamespace(
        anchor_id=anchor_id,
        anchor_type="inspection_anchor",
        target_id=target_id,
        pose=_pose(x, y),
        confidence=confidence,
    )


def test_match_refined_entities_to_live_uses_label_and_distance():
    live = [
        _entity("live_person_0", "person", 1.0, 1.0),
        _entity("live_table_0", "table", 5.0, 5.0),
    ]
    refined = [
        _entity("ref_person_0", "person", 1.2, 0.9),
        _entity("ref_table_0", "table", 9.0, 9.0),
    ]

    matches = match_refined_entities_to_live(live, refined, match_distance_m=0.5, min_confidence=0.5)

    assert matches == {"ref_person_0": "live_person_0"}


def test_enrich_state_with_refinement_adds_reference_only_semantics_without_replacing_entities():
    state = SimpleNamespace(
        source="tb3_semantic_map/live",
        refinement_active=False,
        entities=[_entity("live_person_0", "person", 1.0, 1.0)],
        places=[],
        relations=[],
        anchors=[_anchor("live_anchor_0", "live_person_0", 0.4, 1.0, confidence=0.6)],
    )
    refined_state = SimpleNamespace(
        entities=[_entity("ref_person_0", "person", 1.1, 1.0, confidence=0.9)],
        places=[_place("ref_place_0", ["ref_person_0"], 2.0, 2.0, confidence=0.8)],
        relations=[_relation("ref_person_0", "in_place", "ref_place_0", confidence=0.85)],
        anchors=[_anchor("ref_anchor_0", "ref_person_0", 2.5, 2.0, confidence=0.9)],
    )

    out = enrich_state_with_refinement(
        state,
        refined_state,
        match_distance_m=0.5,
        min_confidence=0.55,
    )

    assert len(out.entities) == 1
    assert "refined_match:ref_person_0" in out.entities[0].provenance
    assert out.source == "tb3_semantic_map/live_with_refinement"
    assert out.refinement_active is True
    assert any(place.place_id == "refined_ref_place_0" for place in out.places)
    assert any(relation.subject_id == "live_person_0" for relation in out.relations)
    assert any(anchor.anchor_id == "refined_ref_anchor_0" for anchor in out.anchors)
