from __future__ import annotations

from datetime import datetime
from typing import Literal

from pydantic import BaseModel, Field


class BBox(BaseModel):
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)


class VideoMetadata(BaseModel):
    path: str
    filename: str
    width: int
    height: int
    duration_s: float
    fps: float | None = None
    frame_count: int | None = None


class Keyframe(BaseModel):
    keyframe_id: str
    image_name: str
    image_path: str
    frame_index: int
    timestamp_s: float
    width: int
    height: int
    registered: bool = False


class CameraPose(BaseModel):
    keyframe_id: str
    image_name: str
    camera_id: int
    center: list[float]
    view_direction: list[float]
    cam_from_world: list[list[float]]
    intrinsics: dict[str, float]


class ReconstructionPoint(BaseModel):
    point_id: int
    xyz: list[float]
    color: list[int] | None = None
    error: float | None = None


class ReconstructionArtifact(BaseModel):
    frame_id: str
    scale_status: str
    alignment_method: str
    camera_model: str
    intrinsics: dict[str, float]
    keyframes: list[Keyframe]
    cameras: list[CameraPose]
    sparse_points: list[ReconstructionPoint]
    support_points: list[ReconstructionPoint] = Field(default_factory=list)
    sparse_ply_path: str | None = None
    scene_extent: dict[str, list[float]] | None = None


class Detection(BaseModel):
    detection_id: str
    keyframe_id: str
    detector_label: str
    confidence: float
    bbox: BBox
    class_distribution: dict[str, float] = Field(default_factory=dict)
    prompt_source: str = "prompt_vocabulary"
    prompt_batch: list[str] = Field(default_factory=list)
    tile_id: str = "full"
    tile_bounds: BBox | None = None
    bbox_area_ratio: float = 0.0
    annotated_image_path: str | None = None


class Tracklet(BaseModel):
    tracklet_id: str
    label: str
    detection_ids: list[str]
    keyframe_ids: list[str]
    confidence: float


class Observation(BaseModel):
    observation_id: str
    keyframe_id: str
    detection_id: str
    detector_label: str
    class_distribution: dict[str, float] = Field(default_factory=dict)
    bbox: BBox
    confidence: float
    world_pose_estimate: list[float]
    pose_covariance: list[list[float]]
    extent_3d: list[float]
    point3d_ids: list[int] = Field(default_factory=list)
    support_point_count: int = 0


class Entity(BaseModel):
    entity_id: str
    canonical_label: str
    class_distribution: dict[str, float]
    aliases: list[str] = Field(default_factory=list)
    ontology_parents: list[str] = Field(default_factory=list)
    world_pose_estimate: list[float]
    pose_covariance: list[list[float]]
    extent_3d: list[float]
    support_surface_id: str | None = None
    place_id: str | None = None
    observed_in_keyframes: list[str]
    observation_ids: list[str]
    support_point3d_ids: list[int] = Field(default_factory=list)
    support_point_count: int = 0
    observation_count: int = 0
    state: Literal["candidate", "confirmed", "occluded", "stale", "lost"] = "candidate"
    mobility: str = "unknown"
    confidence: float


class Place(BaseModel):
    place_id: str
    place_type_distribution: dict[str, float]
    anchor_pose: list[float]
    extent_3d: list[float] = Field(default_factory=list)
    member_entities: list[str]
    support_surfaces: list[str] = Field(default_factory=list)
    observed_in_keyframes: list[str]
    confidence: float


class Relation(BaseModel):
    subject: str
    predicate: str
    object: str
    confidence: float
    evidence_keyframes: list[str]


class TaskAnchor(BaseModel):
    anchor_id: str
    anchor_type: str
    target_id: str
    pose: list[float]
    confidence: float


class SceneGraphArtifact(BaseModel):
    reconstruction: ReconstructionArtifact
    detections: list[Detection]
    tracklets: list[Tracklet]
    observations: list[Observation]
    entities: list[Entity]
    places: list[Place]
    relations: list[Relation]
    task_anchors: list[TaskAnchor]
    prompt_vocabulary: list[str]
    prompt_preset: str = "general"
    yolo_model_size: str = "small"
    scene_profile: str = "general"
    ontology_path: str
    diagnostics: dict[str, str | int | float | bool] = Field(default_factory=dict)
    artifacts: dict[str, str]


class JobRecord(BaseModel):
    job_id: str
    status: Literal["queued", "running", "completed", "failed"]
    current_step: str
    created_at: datetime
    updated_at: datetime
    input_video: str | None = None
    input_video_url: str | None = None
    progress: float = 0.0
    error: str | None = None
    prompt_preset: str | None = None
    model_size: str | None = None
    model_name: str | None = None
    scene_profile: str | None = None
    prompt_count: int = 0
    prompt_preview: list[str] = Field(default_factory=list)
    scene_graph_url: str | None = None
    artifacts: dict[str, str] = Field(default_factory=dict)
