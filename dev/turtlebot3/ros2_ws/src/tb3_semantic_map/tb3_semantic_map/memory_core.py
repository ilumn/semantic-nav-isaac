from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


@dataclass(slots=True)
class SemanticObservation:
    semantic_name: str
    detector_label: str
    confidence: float
    x: float
    y: float
    z: float
    timestamp_sec: float
    covariance_xy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    aliases: list[str] = field(default_factory=list)
    provenance: list[str] = field(default_factory=list)


@dataclass(slots=True)
class SemanticEntityRecord:
    entity_id: str
    semantic_name: str
    detector_label: str
    x: float
    y: float
    z: float
    confidence: float
    observation_count: int
    last_seen: float
    state: str = "active"
    place_id: str = ""
    aliases: list[str] = field(default_factory=list)
    provenance: list[str] = field(default_factory=list)
    extent_xyz: tuple[float, float, float] = (0.25, 0.25, 0.25)
    covariance_xy: tuple[float, float, float] = (0.0, 0.0, 0.0)


class LiveSemanticMemory:
    """Map-frame semantic entity registry with simple matching and aging."""

    def __init__(
        self,
        match_distance_threshold: float = 1.0,
        position_smoothing_alpha: float = 0.3,
        stale_timeout: float = 5.0,
        remove_timeout: float = 30.0,
    ) -> None:
        self.match_distance_threshold = float(match_distance_threshold)
        self.alpha = float(position_smoothing_alpha)
        self.stale_timeout = float(stale_timeout)
        self.remove_timeout = float(remove_timeout)
        self._entities: dict[str, SemanticEntityRecord] = {}
        self._next_id: dict[str, int] = {}

    def update(self, obs: SemanticObservation) -> SemanticEntityRecord:
        matched = self._find_match(obs)
        if matched is not None:
            self._update_existing(matched, obs)
            return matched
        return self._create_new(obs)

    def age(self, current_time: float) -> None:
        to_remove: list[str] = []
        for entity_id, entity in self._entities.items():
            dt = current_time - entity.last_seen
            if dt > self.remove_timeout:
                to_remove.append(entity_id)
            elif dt > self.stale_timeout:
                entity.state = "stale"
            else:
                entity.state = "active"
        for entity_id in to_remove:
            del self._entities[entity_id]

    def get_active_entities(self) -> list[SemanticEntityRecord]:
        return [entity for entity in self._entities.values() if entity.state == "active"]

    def get_all_entities(self) -> list[SemanticEntityRecord]:
        return list(self._entities.values())

    @property
    def size(self) -> int:
        return len(self._entities)

    def _find_match(self, obs: SemanticObservation) -> Optional[SemanticEntityRecord]:
        best: Optional[SemanticEntityRecord] = None
        best_dist = float("inf")
        for entity in self._entities.values():
            if entity.detector_label != obs.detector_label:
                continue
            dist = math.dist((entity.x, entity.y, entity.z), (obs.x, obs.y, obs.z))
            if dist < self.match_distance_threshold and dist < best_dist:
                best = entity
                best_dist = dist
        return best

    def _update_existing(self, entity: SemanticEntityRecord, obs: SemanticObservation) -> None:
        a = self.alpha
        entity.x = a * obs.x + (1.0 - a) * entity.x
        entity.y = a * obs.y + (1.0 - a) * entity.y
        entity.z = a * obs.z + (1.0 - a) * entity.z
        entity.confidence = (
            (entity.confidence * entity.observation_count + obs.confidence)
            / (entity.observation_count + 1)
        )
        entity.covariance_xy = (
            (entity.covariance_xy[0] * entity.observation_count + obs.covariance_xy[0])
            / (entity.observation_count + 1),
            (entity.covariance_xy[1] * entity.observation_count + obs.covariance_xy[1])
            / (entity.observation_count + 1),
            (entity.covariance_xy[2] * entity.observation_count + obs.covariance_xy[2])
            / (entity.observation_count + 1),
        )
        entity.observation_count += 1
        entity.last_seen = obs.timestamp_sec
        entity.state = "active"
        entity.aliases = sorted(set([*entity.aliases, *obs.aliases]))
        entity.provenance = sorted(set([*entity.provenance, *obs.provenance]))

    def _create_new(self, obs: SemanticObservation) -> SemanticEntityRecord:
        seq = self._next_id.get(obs.semantic_name, 0)
        self._next_id[obs.semantic_name] = seq + 1
        entity = SemanticEntityRecord(
            entity_id=f"{obs.semantic_name}_{seq}",
            semantic_name=obs.semantic_name,
            detector_label=obs.detector_label,
            x=obs.x,
            y=obs.y,
            z=obs.z,
            confidence=obs.confidence,
            observation_count=1,
            last_seen=obs.timestamp_sec,
            aliases=list(obs.aliases),
            provenance=list(obs.provenance),
            covariance_xy=obs.covariance_xy,
        )
        self._entities[entity.entity_id] = entity
        return entity
