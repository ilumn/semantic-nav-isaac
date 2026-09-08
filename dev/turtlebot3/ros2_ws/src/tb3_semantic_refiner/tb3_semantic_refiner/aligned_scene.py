from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from .alignment_core import SimilarityTransform2D


@dataclass(slots=True)
class AlignedEntity:
    entity_id: str
    semantic_name: str
    detector_label: str
    place_id: str
    state: str
    aliases: list[str]
    provenance: list[str]
    x: float
    y: float
    z: float
    extent_xyz: tuple[float, float, float]
    confidence: float
    observation_count: int


@dataclass(slots=True)
class AlignedPlace:
    place_id: str
    place_type: str
    member_entity_ids: list[str]
    x: float
    y: float
    z: float
    extent_xyz: tuple[float, float, float]
    confidence: float


@dataclass(slots=True)
class AlignedRelation:
    subject_id: str
    predicate: str
    object_id: str
    confidence: float


@dataclass(slots=True)
class AlignedAnchor:
    anchor_id: str
    anchor_type: str
    target_id: str
    x: float
    y: float
    z: float
    yaw_rad: float
    confidence: float


@dataclass(slots=True)
class AlignedReconstructionPoint:
    point_id: int
    x: float
    y: float
    z: float
    color_rgb: tuple[int, int, int]
    error: float


@dataclass(slots=True)
class AlignedScene:
    job_id: str
    accepted: bool
    alignment: dict
    entities: list[AlignedEntity] = field(default_factory=list)
    places: list[AlignedPlace] = field(default_factory=list)
    relations: list[AlignedRelation] = field(default_factory=list)
    anchors: list[AlignedAnchor] = field(default_factory=list)
    sparse_points: list[AlignedReconstructionPoint] = field(default_factory=list)
    support_points: list[AlignedReconstructionPoint] = field(default_factory=list)
    diagnostics: dict = field(default_factory=dict)


def _load_json(path: Path) -> dict | list:
    return json.loads(path.read_text(encoding="utf-8"))


def _transform_xy(transform: SimilarityTransform2D, pose_xyz: list[float]) -> tuple[float, float, float]:
    x, y = transform.apply(float(pose_xyz[0]), float(pose_xyz[1]))
    z = float(pose_xyz[2]) * abs(transform.scale) if len(pose_xyz) >= 3 else 0.0
    return x, y, z


def _scaled_extent(transform: SimilarityTransform2D, extent_xyz: list[float]) -> tuple[float, float, float]:
    scale = abs(transform.scale)
    if len(extent_xyz) < 3:
        return (0.0, 0.0, 0.0)
    return (
        float(extent_xyz[0]) * scale,
        float(extent_xyz[1]) * scale,
        float(extent_xyz[2]) * scale,
    )


def _anchor_yaw(anchor_xyz: tuple[float, float, float], target_xyz: tuple[float, float, float]) -> float:
    return math.atan2(target_xyz[1] - anchor_xyz[1], target_xyz[0] - anchor_xyz[0])


def _point_color(color_rgb: list[int] | None) -> tuple[int, int, int]:
    if color_rgb is None or len(color_rgb) < 3:
        return (255, 255, 255)
    return (
        max(0, min(255, int(color_rgb[0]))),
        max(0, min(255, int(color_rgb[1]))),
        max(0, min(255, int(color_rgb[2]))),
    )


def _aligned_points(
    transform: SimilarityTransform2D,
    points: list[dict],
) -> list[AlignedReconstructionPoint]:
    aligned_points: list[AlignedReconstructionPoint] = []
    for point in points:
        xyz = list(point.get("xyz", [0.0, 0.0, 0.0]))
        position = _transform_xy(transform, xyz)
        aligned_points.append(
            AlignedReconstructionPoint(
                point_id=int(point.get("point_id", 0)),
                x=position[0],
                y=position[1],
                z=position[2],
                color_rgb=_point_color(point.get("color")),
                error=float(point.get("error", 0.0) or 0.0),
            )
        )
    return aligned_points


def load_aligned_scene(job_dir: str | Path, require_accepted: bool = True) -> AlignedScene | None:
    job_path = Path(job_dir)
    scene_graph_path = job_path / "outputs" / "scene_graph.json"
    alignment_path = job_path / "alignment.json"
    if not scene_graph_path.exists() or not alignment_path.exists():
        return None

    scene_graph = _load_json(scene_graph_path)
    alignment = _load_json(alignment_path)
    accepted = bool(alignment.get("accepted", False))
    if require_accepted and not accepted:
        return None

    transform = SimilarityTransform2D(
        scale=float(alignment.get("scale", 1.0)),
        yaw_rad=float(alignment.get("yaw_rad", 0.0)),
        tx=float(alignment.get("tx", 0.0)),
        ty=float(alignment.get("ty", 0.0)),
    )

    entities: list[AlignedEntity] = []
    entity_positions: dict[str, tuple[float, float, float]] = {}
    for entity in scene_graph.get("entities", []):
        position = _transform_xy(transform, list(entity.get("world_pose_estimate", [0.0, 0.0, 0.0])))
        entity_positions[str(entity.get("entity_id", ""))] = position
        entities.append(
            AlignedEntity(
                entity_id=str(entity.get("entity_id", "")),
                semantic_name=str(entity.get("canonical_label", "")),
                detector_label=str(entity.get("canonical_label", "")),
                place_id=str(entity.get("place_id") or ""),
                state=str(entity.get("state", "confirmed")),
                aliases=[str(item) for item in entity.get("aliases", [])],
                provenance=["semantic_nav_memory", "reference_only_refinement"],
                x=position[0],
                y=position[1],
                z=position[2],
                extent_xyz=_scaled_extent(transform, list(entity.get("extent_3d", [0.0, 0.0, 0.0]))),
                confidence=float(entity.get("confidence", 0.0)),
                observation_count=int(entity.get("observation_count", 0)),
            )
        )

    places: list[AlignedPlace] = []
    for place in scene_graph.get("places", []):
        position = _transform_xy(transform, list(place.get("anchor_pose", [0.0, 0.0, 0.0])))
        distribution = place.get("place_type_distribution", {})
        place_type = "refined_place"
        if distribution:
            place_type = str(max(distribution, key=distribution.get))
        places.append(
            AlignedPlace(
                place_id=str(place.get("place_id", "")),
                place_type=place_type,
                member_entity_ids=[str(item) for item in place.get("member_entities", [])],
                x=position[0],
                y=position[1],
                z=position[2],
                extent_xyz=_scaled_extent(transform, list(place.get("extent_3d", [0.0, 0.0, 0.0]))),
                confidence=float(place.get("confidence", 0.0)),
            )
        )

    relations = [
        AlignedRelation(
            subject_id=str(relation.get("subject", "")),
            predicate=str(relation.get("predicate", "")),
            object_id=str(relation.get("object", "")),
            confidence=float(relation.get("confidence", 0.0)),
        )
        for relation in scene_graph.get("relations", [])
    ]

    anchors: list[AlignedAnchor] = []
    for anchor in scene_graph.get("task_anchors", []):
        anchor_xyz = _transform_xy(transform, list(anchor.get("pose", [0.0, 0.0, 0.0])))
        target_xyz = entity_positions.get(str(anchor.get("target_id", "")), anchor_xyz)
        anchors.append(
            AlignedAnchor(
                anchor_id=str(anchor.get("anchor_id", "")),
                anchor_type=str(anchor.get("anchor_type", "")),
                target_id=str(anchor.get("target_id", "")),
                x=anchor_xyz[0],
                y=anchor_xyz[1],
                z=anchor_xyz[2],
                yaw_rad=_anchor_yaw(anchor_xyz, target_xyz),
                confidence=float(anchor.get("confidence", 0.0)),
            )
        )

    reconstruction = scene_graph.get("reconstruction", {})
    sparse_points = _aligned_points(transform, list(reconstruction.get("sparse_points", [])))
    support_points = _aligned_points(transform, list(reconstruction.get("support_points", [])))

    return AlignedScene(
        job_id=job_path.name,
        accepted=accepted,
        alignment=dict(alignment),
        entities=entities,
        places=places,
        relations=relations,
        anchors=anchors,
        sparse_points=sparse_points,
        support_points=support_points,
        diagnostics=dict(scene_graph.get("diagnostics", {})),
    )
