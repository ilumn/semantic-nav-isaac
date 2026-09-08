#!/usr/bin/env python3
"""
query_core.py — Pure-Python logic for Stage-4 semantic query.

Three responsibilities:
    1. parse_command()    — deterministic text → canonical semantic command
    2. load_target_mapping() — YAML → semantic_name ↔ detector_label dicts
    3. select_target() / select_relational_target() — choose one target from memory

No ROS dependency.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml


# ── Data types ────────────────────────────────────────────────────────────

@dataclass
class MemoryObject:
    """Lightweight mirror of one Detection3D from the memory topic."""
    object_id: str
    detector_label: str
    x: float
    y: float
    confidence: float
    semantic_name: str = ""


@dataclass
class MemoryRelation:
    """Lightweight mirror of one semantic relation from the native semantic map."""
    subject_id: str
    predicate: str
    object_id: str
    confidence: float


@dataclass
class MemoryAnchor:
    """Lightweight mirror of one semantic anchor tied to an entity."""
    anchor_id: str
    anchor_type: str
    target_id: str
    x: float
    y: float
    yaw: float
    confidence: float


@dataclass
class ParsedCommand:
    """Normalized command structure used by the query node."""
    query_text: str
    target_semantic_name: str
    relation_predicate: str = ""
    relation_phrase: str = ""
    reference_semantic_name: str = ""

    @property
    def has_relation(self) -> bool:
        return bool(self.relation_predicate and self.reference_semantic_name)


@dataclass
class QueryResult:
    """Outcome of one semantic query."""
    success: bool
    query_text: str
    semantic_name: str = ""
    detector_label: str = ""
    object_id: str = ""
    x: float = 0.0
    y: float = 0.0
    confidence: float = 0.0
    has_anchor: bool = False
    anchor_id: str = ""
    anchor_type: str = ""
    anchor_x: float = 0.0
    anchor_y: float = 0.0
    anchor_yaw: float = 0.0
    anchor_confidence: float = 0.0
    status_message: str = ""


# ── Mapping loader ────────────────────────────────────────────────────────

def load_target_mapping(yaml_path: str | Path) -> tuple[dict[str, str], dict[str, str]]:
    """Load semantic_targets.yaml and return two lookup dicts.

    Returns:
        (sem2det, det2sem)
        sem2det:  semantic_name  → detector_label
        det2sem:  detector_label → semantic_name
    """
    path = Path(yaml_path)
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    targets = data.get("semantic_targets", [])
    sem2det: dict[str, str] = {}
    det2sem: dict[str, str] = {}
    for entry in targets:
        if not entry.get("enabled", True):
            continue
        sn = entry["semantic_name"]
        dl = entry["detector_label"]
        sem2det[sn] = dl
        det2sem[dl] = sn

    return sem2det, det2sem


# ── Command parser ────────────────────────────────────────────────────────

_FILLER = frozenset({
    "go", "to", "the", "a", "an", "please", "navigate", "approach",
    "find", "me", "take", "bring", "get", "can", "you", "i",
    "want", "need", "would", "like", "could", "help", "with",
})

_PHRASE_ALIASES: dict[str, str] = {
    "bench": "table",
    "stop sign": "stop_sign",
}

_RELATION_ALIASES: dict[str, str] = {
    "next to": "near",
    "left of": "near",
    "right of": "near",
    "near": "near",
}


def _normalize_text(text: str) -> str:
    norm = (text or "").lower().strip()
    norm = re.sub(r"[^\w\s]", " ", norm)
    norm = re.sub(r"\s+", " ", norm).strip()
    return norm


def _replace_phrase_aliases(text: str, aliases: dict[str, str]) -> str:
    result = text
    for phrase, canonical in sorted(aliases.items(), key=lambda item: len(item[0]), reverse=True):
        result = re.sub(rf"\b{re.escape(phrase)}\b", canonical, result)
    return result


def _extract_target(segment: str, known_targets: set[str], prefer_last: bool) -> Optional[str]:
    tokens = segment.split()
    scan = reversed(tokens) if prefer_last else tokens
    for tok in scan:
        if tok in _FILLER:
            continue
        if tok in known_targets:
            return tok
    return None


def _find_relation_phrase(text: str) -> tuple[str, str, str]:
    best_match = None
    for phrase, predicate in _RELATION_ALIASES.items():
        match = re.search(rf"\b{re.escape(phrase)}\b", text)
        if not match:
            continue
        candidate = (match.start(), -len(phrase), phrase, predicate)
        if best_match is None or candidate < best_match:
            best_match = candidate
    if best_match is None:
        return "", "", ""
    _, _, phrase, predicate = best_match
    return phrase, predicate, phrase


def parse_command(text: str, known_targets: set[str]) -> Optional[ParsedCommand]:
    """Parse a text command into target-only or target-reference form."""
    norm = _replace_phrase_aliases(_normalize_text(text), _PHRASE_ALIASES)
    if not norm:
        return None

    relation_phrase, relation_predicate, relation_marker = _find_relation_phrase(norm)
    if relation_phrase:
        match = re.search(rf"\b{re.escape(relation_marker)}\b", norm)
        if match:
            left = norm[: match.start()].strip()
            right = norm[match.end() :].strip()
            target = _extract_target(left, known_targets, prefer_last=True)
            reference = _extract_target(right, known_targets, prefer_last=False)
            if target and reference:
                return ParsedCommand(
                    query_text=text,
                    target_semantic_name=target,
                    relation_predicate=relation_predicate,
                    relation_phrase=relation_phrase,
                    reference_semantic_name=reference,
                )
        return None

    target = _extract_target(norm, known_targets, prefer_last=False)
    if target:
        return ParsedCommand(query_text=text, target_semantic_name=target)
    return None


# ── Target selection ──────────────────────────────────────────────────────

def _matches_target(candidate: MemoryObject, semantic_name: str, detector_label: str) -> bool:
    if candidate.semantic_name and candidate.semantic_name == semantic_name:
        return True
    return candidate.detector_label == detector_label


def _distance_from_robot(candidate: MemoryObject, robot_x: float, robot_y: float) -> float:
    return math.hypot(candidate.x - robot_x, candidate.y - robot_y)


def select_target(
    objects: list[MemoryObject],
    semantic_name: str,
    detector_label: str,
    query_text: str,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
) -> QueryResult:
    """Select the best memory object matching the given detector_label.

    Selection policy: highest confidence (observation_count for persistent
    landmarks), with distance as tiebreaker.
    """
    candidates = [o for o in objects if _matches_target(o, semantic_name, detector_label)]

    if not candidates:
        return QueryResult(
            success=False,
            query_text=query_text,
            semantic_name=semantic_name,
            detector_label=detector_label,
            status_message=f"no active {semantic_name} in memory",
        )

    best = max(
        candidates,
        key=lambda o: (o.confidence, -_distance_from_robot(o, robot_x, robot_y)),
    )
    best_distance = _distance_from_robot(best, robot_x, robot_y)

    return QueryResult(
        success=True,
        query_text=query_text,
        semantic_name=semantic_name,
        detector_label=detector_label,
        object_id=best.object_id,
        x=best.x,
        y=best.y,
        confidence=best.confidence,
        status_message=f"matched {best.object_id} (n={best.confidence:.0f}, "
                        f"d={best_distance:.2f}m, "
                        f"{len(candidates)} candidate(s))",
    )


def _relation_confidence_between(
    relations: list[MemoryRelation],
    target_id: str,
    reference_id: str,
    predicate: str,
) -> float:
    best = 0.0
    for relation in relations:
        if relation.predicate != predicate:
            continue
        direct = relation.subject_id == target_id and relation.object_id == reference_id
        reverse = predicate == "near" and relation.subject_id == reference_id and relation.object_id == target_id
        if direct or reverse:
            best = max(best, float(relation.confidence))
    return best


def select_relational_target(
    objects: list[MemoryObject],
    relations: list[MemoryRelation],
    target_semantic_name: str,
    target_detector_label: str,
    reference_semantic_name: str,
    reference_detector_label: str,
    predicate: str,
    query_text: str,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
) -> QueryResult:
    """Select the best target satisfying a relation to a reference object."""
    target_candidates = [
        candidate
        for candidate in objects
        if _matches_target(candidate, target_semantic_name, target_detector_label)
    ]
    if not target_candidates:
        return QueryResult(
            success=False,
            query_text=query_text,
            semantic_name=target_semantic_name,
            detector_label=target_detector_label,
            status_message=f"no active {target_semantic_name} in memory",
        )

    reference_candidates = [
        candidate
        for candidate in objects
        if _matches_target(candidate, reference_semantic_name, reference_detector_label)
    ]
    if not reference_candidates:
        return QueryResult(
            success=False,
            query_text=query_text,
            semantic_name=target_semantic_name,
            detector_label=target_detector_label,
            status_message=f"no active {reference_semantic_name} in memory",
        )

    best = None
    best_score = None
    best_reference = None
    best_relation_confidence = 0.0

    for target in target_candidates:
        target_distance = _distance_from_robot(target, robot_x, robot_y)
        for reference in reference_candidates:
            if reference.object_id == target.object_id:
                continue
            relation_confidence = _relation_confidence_between(
                relations,
                target.object_id,
                reference.object_id,
                predicate,
            )
            if relation_confidence <= 0.0:
                continue
            score = (
                relation_confidence,
                -target_distance,
                float(target.confidence),
            )
            if best_score is None or score > best_score:
                best_score = score
                best = target
                best_reference = reference
                best_relation_confidence = relation_confidence

    if best is None or best_reference is None:
        return QueryResult(
            success=False,
            query_text=query_text,
            semantic_name=target_semantic_name,
            detector_label=target_detector_label,
            status_message=(
                f"no {predicate} match between {target_semantic_name} "
                f"and {reference_semantic_name}"
            ),
        )

    return QueryResult(
        success=True,
        query_text=query_text,
        semantic_name=target_semantic_name,
        detector_label=target_detector_label,
        object_id=best.object_id,
        x=best.x,
        y=best.y,
        confidence=best.confidence,
        status_message=(
            f"matched {best.object_id} {predicate} {best_reference.object_id} "
            f"(rel={best_relation_confidence:.2f}, d={_distance_from_robot(best, robot_x, robot_y):.2f}m)"
        ),
    )


def select_best_anchor(
    anchors: list[MemoryAnchor],
    target_id: str,
) -> Optional[MemoryAnchor]:
    matches = [anchor for anchor in anchors if anchor.target_id == target_id]
    if not matches:
        return None
    return max(matches, key=lambda anchor: anchor.confidence)


def attach_anchor(
    result: QueryResult,
    anchor: Optional[MemoryAnchor],
) -> QueryResult:
    if not result.success or anchor is None:
        return result

    result.has_anchor = True
    result.anchor_id = anchor.anchor_id
    result.anchor_type = anchor.anchor_type
    result.anchor_x = anchor.x
    result.anchor_y = anchor.y
    result.anchor_yaw = anchor.yaw
    result.anchor_confidence = anchor.confidence
    result.status_message = (
        f"{result.status_message}; anchor={anchor.anchor_id} "
        f"(conf={anchor.confidence:.2f})"
    )
    return result
