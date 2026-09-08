"""Pure helpers for evaluating the Isaac Sim ROS graph contract."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Iterable, Optional


@dataclass(frozen=True, slots=True)
class TopicContract:
    """Expected ROS graph and message-rate properties for one topic."""

    name: str
    type_name: str
    min_publishers: int = 1
    max_publishers: Optional[int] = 1
    min_subscribers: int = 0
    min_rate_hz: Optional[float] = None
    max_age_sec: Optional[float] = None
    require_sample: bool = True


@dataclass(frozen=True, slots=True)
class TopicObservation:
    """Observed endpoint and traffic properties for one topic."""

    publisher_count: int = 0
    subscriber_count: int = 0
    publisher_types: frozenset[str] = frozenset()
    subscriber_types: frozenset[str] = frozenset()
    sample_count: int = 0
    rate_hz: Optional[float] = None
    age_sec: Optional[float] = None


@dataclass(frozen=True, slots=True)
class ContractViolation:
    """One machine-readable contract failure."""

    topic: str
    code: str
    detail: str


@dataclass(frozen=True, slots=True)
class RateSample:
    """Traffic statistics for a bounded monotonic-time window."""

    sample_count: int
    rate_hz: Optional[float]
    age_sec: Optional[float]


@dataclass(frozen=True, slots=True)
class TfRequirement:
    """A target/source transform which must be available."""

    label: str
    target_frame: str
    source_frame: str


class RateWindow:
    """Track per-topic arrival times using an injected monotonic clock."""

    def __init__(self, window_sec: float) -> None:
        self.window_sec = max(0.1, float(window_sec))
        self._samples: dict[str, deque[float]] = defaultdict(deque)
        self._last_seen: dict[str, float] = {}
        self._total_counts: dict[str, int] = defaultdict(int)

    def record(self, topic: str, timestamp: float) -> None:
        samples = self._samples[topic]
        timestamp = float(timestamp)
        samples.append(timestamp)
        self._last_seen[topic] = timestamp
        self._total_counts[topic] += 1
        self._purge(samples, timestamp)

    def sample(self, topic: str, now: float) -> RateSample:
        samples = self._samples[topic]
        now = float(now)
        self._purge(samples, now)
        last_seen = self._last_seen.get(topic)
        if last_seen is None:
            return RateSample(0, None, None)

        age_sec = max(0.0, now - last_seen)
        rate_hz: Optional[float] = None
        if len(samples) >= 2:
            duration = samples[-1] - samples[0]
            if duration > 0.0:
                rate_hz = (len(samples) - 1) / duration
        return RateSample(self._total_counts[topic], rate_hz, age_sec)

    def _purge(self, samples: deque[float], now: float) -> None:
        cutoff = now - self.window_sec
        while samples and samples[0] < cutoff:
            samples.popleft()


def evaluate_topic_contract(
    contract: TopicContract,
    observation: TopicObservation,
    *,
    in_grace_period: bool = False,
) -> list[ContractViolation]:
    """Return all violations for a topic observation.

    Missing endpoints and traffic are suppressed during startup grace, while
    duplicate publishers and known type mismatches are reported immediately.
    """

    violations: list[ContractViolation] = []

    if not in_grace_period and observation.publisher_count < contract.min_publishers:
        violations.append(
            ContractViolation(
                contract.name,
                "publisher_count_low",
                f"expected >= {contract.min_publishers}, got {observation.publisher_count}",
            )
        )
    if (
        contract.max_publishers is not None
        and observation.publisher_count > contract.max_publishers
    ):
        violations.append(
            ContractViolation(
                contract.name,
                "publisher_count_high",
                f"expected <= {contract.max_publishers}, got {observation.publisher_count}",
            )
        )
    if not in_grace_period and observation.subscriber_count < contract.min_subscribers:
        violations.append(
            ContractViolation(
                contract.name,
                "subscriber_count_low",
                f"expected >= {contract.min_subscribers}, got {observation.subscriber_count}",
            )
        )

    endpoint_types = observation.publisher_types
    if contract.min_publishers == 0 and contract.min_subscribers > 0:
        endpoint_types = observation.subscriber_types
    if endpoint_types and contract.type_name not in endpoint_types:
        violations.append(
            ContractViolation(
                contract.name,
                "type_mismatch",
                f"expected {contract.type_name}, got {sorted(endpoint_types)}",
            )
        )

    if in_grace_period or not contract.require_sample:
        return violations

    if observation.sample_count == 0:
        violations.append(
            ContractViolation(contract.name, "no_samples", "no messages observed")
        )
        return violations

    if contract.min_rate_hz is not None:
        if observation.rate_hz is None:
            violations.append(
                ContractViolation(
                    contract.name,
                    "insufficient_rate_samples",
                    "at least two samples are required to estimate rate",
                )
            )
        elif observation.rate_hz < contract.min_rate_hz:
            violations.append(
                ContractViolation(
                    contract.name,
                    "rate_low",
                    f"expected >= {contract.min_rate_hz:.2f} Hz, "
                    f"got {observation.rate_hz:.2f} Hz",
                )
            )

    if (
        contract.max_age_sec is not None
        and observation.age_sec is not None
        and observation.age_sec > contract.max_age_sec
    ):
        violations.append(
            ContractViolation(
                contract.name,
                "stale",
                f"last message age {observation.age_sec:.2f}s exceeds "
                f"{contract.max_age_sec:.2f}s",
            )
        )

    return violations


def evaluate_contracts(
    contracts: Iterable[TopicContract],
    observations: dict[str, TopicObservation],
    *,
    in_grace_period: bool = False,
) -> list[ContractViolation]:
    """Evaluate a set of topic contracts."""

    violations: list[ContractViolation] = []
    for contract in contracts:
        observation = observations.get(contract.name, TopicObservation())
        violations.extend(
            evaluate_topic_contract(
                contract,
                observation,
                in_grace_period=in_grace_period,
            )
        )
    return violations


def required_tf_chain(
    *,
    map_frame: str,
    odom_frame: str,
    base_footprint_frame: str,
    base_frame: str,
    scan_frame: str,
    camera_frame: str,
    camera_optical_frame: str,
    require_map_frame: bool = True,
    require_camera_optical_frame: bool = True,
) -> list[TfRequirement]:
    """Build the transform requirements shared by the checker and tests."""

    requirements: list[TfRequirement] = []
    if require_map_frame:
        requirements.append(
            TfRequirement("map_to_odom", map_frame, odom_frame)
        )
    requirements.extend(
        [
            TfRequirement(
                "odom_to_base_footprint",
                odom_frame,
                base_footprint_frame,
            ),
            TfRequirement(
                "base_footprint_to_base_link",
                base_footprint_frame,
                base_frame,
            ),
            TfRequirement("base_link_to_scan", base_frame, scan_frame),
            TfRequirement("base_link_to_camera", base_frame, camera_frame),
        ]
    )
    if require_camera_optical_frame:
        requirements.append(
            TfRequirement(
                "camera_to_camera_optical",
                camera_frame,
                camera_optical_frame,
            )
        )
    return requirements
