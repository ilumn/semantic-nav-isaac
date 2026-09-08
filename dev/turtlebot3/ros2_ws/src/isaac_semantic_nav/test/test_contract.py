"""Unit tests for the ROS-independent Isaac contract helpers."""

import pytest

from isaac_semantic_nav.contract import (
    RateWindow,
    TopicContract,
    TopicObservation,
    evaluate_topic_contract,
    required_tf_chain,
)


def test_rate_window_estimates_rate_and_age() -> None:
    window = RateWindow(5.0)
    window.record("/scan", 10.0)
    window.record("/scan", 10.5)
    window.record("/scan", 11.0)

    sample = window.sample("/scan", 11.25)

    assert sample.sample_count == 3
    assert sample.rate_hz == pytest.approx(2.0)
    assert sample.age_sec == pytest.approx(0.25)


def test_rate_window_remembers_latched_sample_after_rate_window() -> None:
    window = RateWindow(1.0)
    window.record("/tf_static", 2.0)

    sample = window.sample("/tf_static", 10.0)

    assert sample.sample_count == 1
    assert sample.rate_hz is None
    assert sample.age_sec == pytest.approx(8.0)


def test_missing_endpoint_and_samples_are_suppressed_during_grace() -> None:
    contract = TopicContract(
        "/scan",
        "sensor_msgs/msg/LaserScan",
        min_rate_hz=5.0,
    )

    assert evaluate_topic_contract(
        contract,
        TopicObservation(),
        in_grace_period=True,
    ) == []


def test_duplicate_clock_publisher_is_reported_during_grace() -> None:
    contract = TopicContract("/clock", "rosgraph_msgs/msg/Clock")
    observation = TopicObservation(
        publisher_count=2,
        publisher_types=frozenset({"rosgraph_msgs/msg/Clock"}),
    )

    violations = evaluate_topic_contract(
        contract,
        observation,
        in_grace_period=True,
    )

    assert [violation.code for violation in violations] == [
        "publisher_count_high"
    ]


def test_low_rate_and_stale_stream_are_reported() -> None:
    contract = TopicContract(
        "/odom",
        "nav_msgs/msg/Odometry",
        min_rate_hz=10.0,
        max_age_sec=1.0,
    )
    observation = TopicObservation(
        publisher_count=1,
        publisher_types=frozenset({"nav_msgs/msg/Odometry"}),
        sample_count=4,
        rate_hz=4.0,
        age_sec=3.0,
    )

    violations = evaluate_topic_contract(contract, observation)

    assert {violation.code for violation in violations} == {"rate_low", "stale"}


def test_cmd_vel_requires_unstamped_twist_subscriber_without_test_publish() -> None:
    contract = TopicContract(
        "/cmd_vel",
        "geometry_msgs/msg/Twist",
        min_publishers=0,
        max_publishers=None,
        min_subscribers=1,
        require_sample=False,
    )
    observation = TopicObservation(
        publisher_count=0,
        subscriber_count=1,
        subscriber_types=frozenset({"geometry_msgs/msg/Twist"}),
    )

    assert evaluate_topic_contract(contract, observation) == []


def test_cmd_vel_rejects_twist_stamped_subscriber() -> None:
    contract = TopicContract(
        "/cmd_vel",
        "geometry_msgs/msg/Twist",
        min_publishers=0,
        max_publishers=None,
        min_subscribers=1,
        require_sample=False,
    )
    observation = TopicObservation(
        subscriber_count=1,
        subscriber_types=frozenset({"geometry_msgs/msg/TwistStamped"}),
    )

    violations = evaluate_topic_contract(contract, observation)

    assert [violation.code for violation in violations] == ["type_mismatch"]


def test_required_tf_chain_can_omit_optional_edges() -> None:
    requirements = required_tf_chain(
        map_frame="map",
        odom_frame="odom",
        base_footprint_frame="base_footprint",
        base_frame="base_link",
        scan_frame="base_scan",
        camera_frame="camera_rgb_frame",
        camera_optical_frame="camera_rgb_optical_frame",
        require_map_frame=False,
        require_camera_optical_frame=False,
    )

    assert [requirement.label for requirement in requirements] == [
        "odom_to_base_footprint",
        "base_footprint_to_base_link",
        "base_link_to_scan",
        "base_link_to_camera",
    ]
