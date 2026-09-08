from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    from tb3_semantic_map.compat import state_to_detection3d_array
except ImportError as exc:
    pytest.skip(f"ROS message modules not available for compatibility tests: {exc}", allow_module_level=True)


def test_state_to_detection3d_array_preserves_ids_labels_and_frame():
    header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2), frame_id="map")
    pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=0.0))
    entity = SimpleNamespace(
        header=header,
        entity_id="person_0",
        detector_label="person",
        confidence=0.9,
        pose=pose,
    )
    state = SimpleNamespace(header=header, entities=[entity])

    msg = state_to_detection3d_array(state)

    assert msg.header.frame_id == "map"
    assert len(msg.detections) == 1
    assert msg.detections[0].id == "person_0"
    assert msg.detections[0].results[0].hypothesis.class_id == "person"
