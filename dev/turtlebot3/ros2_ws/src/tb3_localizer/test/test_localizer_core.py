import math

from tb3_localizer.localizer_core import LocalizerCore


def test_bearing_index_supports_zero_to_two_pi_scan():
    core = LocalizerCore()
    increment = (2.0 * math.pi) / 360.0

    assert core.bearing_to_scan_index(
        -math.radians(10.0), 0.0, 2.0 * math.pi - increment, increment, 360
    ) == 350


def test_bearing_index_supports_minus_pi_to_pi_scan():
    core = LocalizerCore()
    increment = (2.0 * math.pi) / 360.0

    assert core.bearing_to_scan_index(
        -math.radians(10.0), -math.pi, math.pi - increment, increment, 360
    ) == 170


def test_bearing_outside_partial_scan_returns_none():
    core = LocalizerCore()

    assert core.bearing_to_scan_index(2.0, -1.0, 1.0, 0.01, 201) is None


def test_full_circle_range_window_wraps_across_scan_boundary():
    core = LocalizerCore(scan_window_half=1, min_valid_range=0.1, max_valid_range=5.0)
    ranges = [1.0, math.inf, math.inf, 3.0]

    assert core.robust_range(ranges, 0, wrap_around=True) == 3.0


def test_localize_accepts_isaac_style_minus_pi_to_pi_scan():
    core = LocalizerCore(scan_window_half=0)
    increment = (2.0 * math.pi) / 360.0
    ranges = [math.inf] * 360
    ranges[180] = 2.0

    localized = core.localize(
        label="person",
        confidence=0.9,
        bbox_center_x=320.0,
        image_width=640,
        scan_ranges=ranges,
        scan_angle_min=-math.pi,
        scan_angle_max=math.pi - increment,
        scan_angle_increment=increment,
    )

    assert localized is not None
    assert localized.scan_index == 180
    assert localized.x == 2.0
    assert abs(localized.y) < 1e-12
