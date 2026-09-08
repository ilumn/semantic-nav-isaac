from __future__ import annotations

from pathlib import Path

from .models import Entity, Place, Relation, SceneGraphArtifact, TaskAnchor
from .models import Detection, Observation, ReconstructionArtifact, Tracklet


def build_task_anchors(entities: list[Entity]) -> list[TaskAnchor]:
    anchors: list[TaskAnchor] = []
    for index, entity in enumerate(entities, start=1):
        offset = max(entity.extent_3d[0], entity.extent_3d[1], 0.1) * 1.5
        pose = [
            entity.world_pose_estimate[0] + offset,
            entity.world_pose_estimate[1],
            entity.world_pose_estimate[2] + entity.extent_3d[2] * 0.25,
        ]
        anchors.append(
            TaskAnchor(
                anchor_id=f"inspection_anchor_{index:02d}",
                anchor_type="inspection_anchor",
                target_id=entity.entity_id,
                pose=pose,
                confidence=min(0.9, 0.45 + entity.confidence * 0.35),
            )
        )
    return anchors


def assemble_scene_graph(
    reconstruction: ReconstructionArtifact,
    detections: list[Detection],
    tracklets: list[Tracklet],
    observations: list[Observation],
    entities: list[Entity],
    places: list[Place],
    relations: list[Relation],
    task_anchors: list[TaskAnchor],
    prompt_vocabulary: list[str],
    prompt_preset: str,
    yolo_model_size: str,
    scene_profile: str,
    ontology_path: Path,
    diagnostics: dict[str, str | int | float | bool],
    artifacts: dict[str, str],
) -> SceneGraphArtifact:
    return SceneGraphArtifact(
        reconstruction=reconstruction,
        detections=detections,
        tracklets=tracklets,
        observations=observations,
        entities=entities,
        places=places,
        relations=relations,
        task_anchors=task_anchors,
        prompt_vocabulary=prompt_vocabulary,
        prompt_preset=prompt_preset,
        yolo_model_size=yolo_model_size,
        scene_profile=scene_profile,
        ontology_path=str(ontology_path),
        diagnostics=diagnostics,
        artifacts=artifacts,
    )
