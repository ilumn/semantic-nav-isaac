from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Callable

from .config import PipelineConfig
from .detector_yoloworld import YOLOWorldDetector
from .entity_memory import fuse_entities
from .exporters import write_json
from .geometry_colmap import COLMAPGeometryBackend
from .grounder_3d import ground_detections
from .models import ReconstructionPoint, SceneGraphArtifact, VideoMetadata
from .ontology import Ontology
from .place_builder import build_places
from .relation_inference import infer_relations
from .scene_graph import assemble_scene_graph, build_task_anchors
from .tracker_2d import build_tracklets
from .video_io import (
    extract_keyframes,
    extract_keyframes_from_sequence,
    probe_image_sequence,
    probe_video,
)


ProgressCallback = Callable[[str, float], None]


def _artifact_url(job_id: str, relative_path: Path) -> str:
    return f"/jobs/{job_id}/{relative_path.as_posix()}"


def _apply_camera_prior_from_manifest(config: PipelineConfig, frame_manifest_path: Path | None) -> None:
    if frame_manifest_path is None or not frame_manifest_path.exists():
        return
    if all(
        value is not None
        for value in (
            config.camera_fx,
            config.camera_fy,
            config.camera_cx,
            config.camera_cy,
        )
    ):
        return

    payload = json.loads(frame_manifest_path.read_text(encoding="utf-8"))
    for frame in payload.get("frames", []):
        fx = frame.get("fx")
        fy = frame.get("fy")
        cx = frame.get("cx")
        cy = frame.get("cy")
        if None in (fx, fy, cx, cy):
            continue
        config.camera_fx = float(fx)
        config.camera_fy = float(fy)
        config.camera_cx = float(cx)
        config.camera_cy = float(cy)
        if config.camera_width is None and frame.get("width") is not None:
            config.camera_width = int(frame["width"])
        if config.camera_height is None and frame.get("height") is not None:
            config.camera_height = int(frame["height"])
        return


def run_pipeline(
    job_id: str,
    job_dir: Path,
    config: PipelineConfig,
    progress: ProgressCallback,
    *,
    video_path: Path | None = None,
    image_dir: Path | None = None,
    frame_manifest_path: Path | None = None,
) -> SceneGraphArtifact:
    if (video_path is None) == (image_dir is None):
        raise ValueError("Provide exactly one input source: video_path or image_dir.")

    ontology = Ontology.from_default_file()
    metadata: VideoMetadata

    inputs_dir = job_dir / "input"
    work_dir = job_dir / "work"
    outputs_dir = job_dir / "outputs"
    frames_dir = work_dir / "images"
    detections_dir = outputs_dir / "annotated_frames"
    for directory in (inputs_dir, work_dir, outputs_dir, detections_dir):
        directory.mkdir(parents=True, exist_ok=True)

    if image_dir is not None:
        _apply_camera_prior_from_manifest(config, frame_manifest_path)
        progress("probing_image_sequence", 0.06)
        metadata = probe_image_sequence(image_dir, frame_manifest_path=frame_manifest_path)
        write_json(outputs_dir / "video_metadata.json", metadata)

        progress("preparing_keyframes", 0.14)
        keyframes = extract_keyframes_from_sequence(
            image_dir=image_dir,
            output_dir=frames_dir,
            max_keyframes=config.max_keyframes,
            max_image_size=config.max_image_size,
            frame_manifest_path=frame_manifest_path,
        )
        input_mode = "image_sequence"
    else:
        assert video_path is not None
        canonical_video_path = inputs_dir / video_path.name
        if video_path != canonical_video_path:
            shutil.copy2(video_path, canonical_video_path)
            video_path = canonical_video_path

        progress("probing_video", 0.06)
        metadata = probe_video(video_path)
        write_json(outputs_dir / "video_metadata.json", metadata)

        progress("extracting_keyframes", 0.14)
        keyframes = extract_keyframes(
            video_path=video_path,
            output_dir=frames_dir,
            sample_fps=config.frame_sample_fps,
            max_keyframes=config.max_keyframes,
            max_image_size=config.max_image_size,
        )
        input_mode = "video"

    progress("reconstructing_geometry", 0.4)
    geometry = COLMAPGeometryBackend(config).reconstruct(keyframes=keyframes, workspace_dir=work_dir)

    progress("running_yolo_world", 0.58)
    detections = YOLOWorldDetector(config).detect(keyframes=geometry.artifact.keyframes, output_dir=detections_dir)
    write_json(outputs_dir / "detections.json", [item.model_dump(mode="json") for item in detections])

    progress("building_tracklets", 0.66)
    tracklets = build_tracklets(detections)
    write_json(outputs_dir / "tracklets.json", [item.model_dump(mode="json") for item in tracklets])

    progress("grounding_observations", 0.76)
    observations = ground_detections(detections, geometry)
    write_json(outputs_dir / "observations.json", [item.model_dump(mode="json") for item in observations])
    support_point_ids = sorted({point_id for observation in observations for point_id in observation.point3d_ids})
    geometry.artifact.support_points = [
        ReconstructionPoint(
            point_id=point_id,
            xyz=geometry.point_lookup[point_id].astype(float).tolist(),
            color=[int(channel) for channel in geometry.reconstruction.point3D(point_id).color],
            error=float(geometry.reconstruction.point3D(point_id).error)
            if geometry.reconstruction.point3D(point_id).has_error()
            else None,
        )
        for point_id in support_point_ids
        if point_id in geometry.point_lookup
    ]
    write_json(outputs_dir / "reconstruction.json", geometry.artifact)

    progress("fusing_entities", 0.84)
    entities = fuse_entities(
        observations=observations,
        tracklets=tracklets,
        ontology=ontology,
        scene_diagonal=geometry.scene_diagonal,
        config=config,
    )
    write_json(outputs_dir / "entities.json", [item.model_dump(mode="json") for item in entities])

    progress("inferring_places", 0.9)
    places = build_places(
        entities=entities,
        ontology=ontology,
        scene_diagonal=geometry.scene_diagonal,
        config=config,
    )
    write_json(outputs_dir / "places.json", [item.model_dump(mode="json") for item in places])

    progress("inferring_relations", 0.95)
    relations = infer_relations(
        entities=entities,
        places=places,
        ontology=ontology,
        scene_diagonal=geometry.scene_diagonal,
        config=config,
    )
    write_json(outputs_dir / "relations.json", [item.model_dump(mode="json") for item in relations])

    progress("building_scene_graph", 0.98)
    task_anchors = build_task_anchors(entities)
    write_json(outputs_dir / "task_anchors.json", [item.model_dump(mode="json") for item in task_anchors])

    diagnostics: dict[str, str | int | float | bool] = {
        "registered_keyframes": geometry.registered_keyframes,
        "registered_keyframe_ratio": round(geometry.registered_keyframe_ratio, 4),
        "total_keyframes": len(geometry.artifact.keyframes),
        "detections": len(detections),
        "observations": len(observations),
        "entities": len(entities),
        "places": len(places),
        "relations": len(relations),
        "prompt_count": len(config.prompt_vocabulary),
        "yolo_model_size": config.yolo_model_size,
        "yolo_model_name": Path(config.yolo_model_name).name,
        "scene_diagonal": round(geometry.scene_diagonal, 4),
        "reconstruction_strategy": geometry.reconstruction_strategy,
        "reconstruction_attempts": "; ".join(geometry.attempt_summaries),
        "input_mode": input_mode,
    }

    artifacts = {
        "video_metadata": _artifact_url(job_id, Path("outputs/video_metadata.json")),
        "reconstruction": _artifact_url(job_id, Path("outputs/reconstruction.json")),
        "detections": _artifact_url(job_id, Path("outputs/detections.json")),
        "tracklets": _artifact_url(job_id, Path("outputs/tracklets.json")),
        "observations": _artifact_url(job_id, Path("outputs/observations.json")),
        "entities": _artifact_url(job_id, Path("outputs/entities.json")),
        "places": _artifact_url(job_id, Path("outputs/places.json")),
        "relations": _artifact_url(job_id, Path("outputs/relations.json")),
        "task_anchors": _artifact_url(job_id, Path("outputs/task_anchors.json")),
        "annotated_frames": _artifact_url(job_id, Path("outputs/annotated_frames")),
    }
    scene_graph = assemble_scene_graph(
        reconstruction=geometry.artifact,
        detections=detections,
        tracklets=tracklets,
        observations=observations,
        entities=entities,
        places=places,
        relations=relations,
        task_anchors=task_anchors,
        prompt_vocabulary=config.prompt_vocabulary,
        prompt_preset=config.prompt_preset,
        yolo_model_size=config.yolo_model_size,
        scene_profile=config.scene_profile,
        ontology_path=ontology.path,
        diagnostics=diagnostics,
        artifacts=artifacts,
    )
    write_json(outputs_dir / "scene_graph.json", scene_graph)

    progress("completed", 1.0)
    return scene_graph
