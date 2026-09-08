from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tb3_semantic_refiner.bundle_io import BundleWriter


def test_bundle_writer_persists_manifest_and_frames(tmp_path):
    writer = BundleWriter(str(tmp_path), max_frames=2)
    image = np.zeros((8, 8, 3), dtype=np.uint8)

    path0 = writer.write_frame(0, image, {"stamp_sec": 1.0})
    path1 = writer.write_frame(1, image, {"stamp_sec": 2.0})

    manifest = json.loads((tmp_path / "latest" / "manifest.json").read_text(encoding="utf-8"))
    assert Path(path0).exists()
    assert Path(path1).exists()
    assert len(manifest["frames"]) == 2


def test_bundle_writer_evicts_oldest_frame(tmp_path):
    writer = BundleWriter(str(tmp_path), max_frames=2)
    image = np.zeros((8, 8, 3), dtype=np.uint8)

    path0 = writer.write_frame(0, image, {"stamp_sec": 1.0})
    writer.write_frame(1, image, {"stamp_sec": 2.0})
    writer.write_frame(2, image, {"stamp_sec": 3.0})

    manifest = json.loads((tmp_path / "latest" / "manifest.json").read_text(encoding="utf-8"))
    assert not Path(path0).exists()
    assert len(manifest["frames"]) == 2
