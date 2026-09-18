from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SRC_ROOT / "tb3_detector"))

from tb3_semantic_map import detector_bridge as detector_bridge_module


class _FakeDetectorCore:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.is_loaded = False

    def load(self) -> None:
        self.is_loaded = True

    def infer(self, bgr_image):
        return [{"label": "person", "conf": 0.9, "bbox_xyxy": [0.0, 0.0, 10.0, 10.0]}]


def test_detector_bridge_proxies_load_and_infer(monkeypatch):
    monkeypatch.setattr(detector_bridge_module, "DetectorCore", _FakeDetectorCore)

    bridge = detector_bridge_module.DetectorBridge(
        model_id="nvidia/LocateAnything-3B",
        model_revision="test-revision",
        class_filter=["person"],
        device="cpu",
        generation_mode="hybrid",
        max_new_tokens=256,
        local_files_only=True,
    )

    assert bridge.is_loaded is False
    bridge.load()
    assert bridge.is_loaded is True
    detections = bridge.infer(object())
    assert detections[0]["label"] == "person"
    assert bridge._core.kwargs["model_revision"] == "test-revision"
