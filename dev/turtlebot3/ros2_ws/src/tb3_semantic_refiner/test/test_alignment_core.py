from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tb3_semantic_refiner.alignment_core import estimate_alignment_2d, estimate_similarity_transform_2d


def test_estimate_similarity_transform_2d_recovers_scale_rotation_translation():
    source = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    target = [(2.0, 3.0), (4.0, 3.0), (2.0, 5.0)]

    transform = estimate_similarity_transform_2d(source, target)

    assert abs(transform.scale - 2.0) < 1e-6
    assert abs(transform.yaw_rad - 0.0) < 1e-6
    assert abs(transform.tx - 2.0) < 1e-6
    assert abs(transform.ty - 3.0) < 1e-6


def test_estimate_similarity_transform_2d_recovers_rotation():
    source = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    target = [(5.0, -1.0), (5.0, 0.0), (4.0, -1.0)]

    transform = estimate_similarity_transform_2d(source, target)
    out = transform.apply(1.0, 0.0)

    assert abs(transform.scale - 1.0) < 1e-6
    assert abs(transform.yaw_rad - math.pi / 2.0) < 1e-6
    assert abs(out[0] - 5.0) < 1e-6
    assert abs(out[1] - 0.0) < 1e-6


def test_estimate_alignment_2d_reports_confident_accepted_solution():
    source = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    target = [(2.0, 3.0), (4.0, 3.0), (2.0, 5.0)]

    estimate = estimate_alignment_2d(source, target)

    assert estimate.accepted is True
    assert estimate.correspondences == 3
    assert estimate.rms_error < 1e-6
    assert estimate.confidence > 0.3
