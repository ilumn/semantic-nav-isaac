from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SRC_ROOT / "tb3_localizer"))

from tb3_semantic_map.memory_core import LiveSemanticMemory, SemanticObservation


def test_memory_merges_same_label_within_threshold():
    memory = LiveSemanticMemory(match_distance_threshold=1.0, position_smoothing_alpha=0.5)
    first = memory.update(
        SemanticObservation(
            "person",
            "person",
            0.8,
            1.0,
            2.0,
            0.0,
            1.0,
            covariance_xy=(0.1, 0.01, 0.2),
            provenance=["live"],
        )
    )
    second = memory.update(
        SemanticObservation(
            "person",
            "person",
            0.6,
            1.2,
            2.2,
            0.0,
            2.0,
            covariance_xy=(0.2, 0.02, 0.3),
            provenance=["live"],
        )
    )

    assert first.entity_id == second.entity_id
    assert memory.size == 1
    assert second.observation_count == 2
    assert second.last_seen == 2.0
    assert second.covariance_xy[0] > 0.1
    assert second.covariance_xy[2] > 0.2


def test_memory_ages_and_removes_entities():
    memory = LiveSemanticMemory(stale_timeout=5.0, remove_timeout=10.0)
    entity = memory.update(
        SemanticObservation("table", "bench", 0.9, 0.0, 0.0, 0.0, 0.0, provenance=["live"])
    )

    memory.age(6.0)
    assert memory.get_all_entities()[0].state == "stale"

    memory.age(11.0)
    assert memory.size == 0
