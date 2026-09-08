from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import pycolmap

from .config import PipelineConfig
from .models import CameraPose, Keyframe, ReconstructionArtifact, ReconstructionPoint


TransformFn = Callable[[np.ndarray], np.ndarray]


@dataclass(slots=True)
class GeometryResult:
    reconstruction: pycolmap.Reconstruction
    artifact: ReconstructionArtifact
    point_lookup: dict[int, np.ndarray]
    transform_point: TransformFn
    scene_diagonal: float
    registered_keyframes: int
    registered_keyframe_ratio: float
    reconstruction_strategy: str
    attempt_summaries: list[str]


@dataclass(slots=True)
class ReconstructionAttempt:
    label: str
    reconstruction: pycolmap.Reconstruction | None
    registered_names: set[str]
    summary: str
    camera_model: str
    error: str | None = None


@dataclass(slots=True)
class AttemptConfig:
    label: str
    matcher: str
    camera_model: str
    use_camera_prior: bool = False
    aggressive: bool = False


def _as_list(value) -> list[float]:
    return np.asarray(value, dtype=float).reshape(-1).tolist()


def _pca_transform(camera_centers: np.ndarray, sparse_points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    reference = camera_centers if len(camera_centers) >= 3 else sparse_points
    if len(reference) == 0:
        return np.zeros(3), np.eye(3)

    centroid = reference.mean(axis=0)
    centered = reference - centroid
    if len(reference) < 3 or np.allclose(centered, 0):
        return centroid, np.eye(3)

    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    rotation = vh.T
    if np.linalg.det(rotation) < 0:
        rotation[:, -1] *= -1
    return centroid, rotation


def _transformer(centroid: np.ndarray, rotation: np.ndarray) -> TransformFn:
    def transform_point(xyz: np.ndarray) -> np.ndarray:
        return (np.asarray(xyz, dtype=float) - centroid) @ rotation

    return transform_point


class COLMAPGeometryBackend:
    def __init__(self, config: PipelineConfig):
        self.config = config

    def _camera_prior_available(self) -> bool:
        return all(
            value is not None
            for value in (
                self.config.camera_fx,
                self.config.camera_fy,
                self.config.camera_cx,
                self.config.camera_cy,
            )
        )

    def _camera_params(self, camera_model: str) -> str:
        if not self._camera_prior_available():
            return ""
        fx = float(self.config.camera_fx or 0.0)
        fy = float(self.config.camera_fy or fx)
        cx = float(self.config.camera_cx or 0.0)
        cy = float(self.config.camera_cy or 0.0)
        if camera_model == "PINHOLE":
            return f"{fx},{fy},{cx},{cy}"
        if camera_model == "SIMPLE_PINHOLE":
            focal = 0.5 * (fx + fy)
            return f"{focal},{cx},{cy}"
        if camera_model == "SIMPLE_RADIAL":
            focal = 0.5 * (fx + fy)
            return f"{focal},{cx},{cy},0.0"
        return ""

    def _feature_extraction_options(self, aggressive: bool) -> pycolmap.FeatureExtractionOptions:
        options = pycolmap.FeatureExtractionOptions()
        options.use_gpu = False
        options.max_image_size = -1
        options.sift.max_num_features = max(8192, int(self.config.reconstruction_sift_max_num_features))
        options.sift.peak_threshold = min(
            options.sift.peak_threshold,
            max(1e-4, float(self.config.reconstruction_sift_peak_threshold)),
        )
        options.sift.domain_size_pooling = bool(self.config.reconstruction_use_domain_size_pooling)
        options.sift.estimate_affine_shape = bool(self.config.reconstruction_use_affine_shape)
        if aggressive:
            options.sift.max_num_features = max(options.sift.max_num_features, 28000)
            options.sift.peak_threshold = min(options.sift.peak_threshold, 0.0025)
            options.sift.domain_size_pooling = True
            options.sift.estimate_affine_shape = True
            options.sift.darkness_adaptivity = True
            options.sift.dsp_num_scales = max(int(options.sift.dsp_num_scales), 16)
        return options

    def _matching_options(self, aggressive: bool) -> pycolmap.FeatureMatchingOptions:
        options = pycolmap.FeatureMatchingOptions()
        options.guided_matching = True
        options.use_gpu = False
        options.max_num_matches = max(int(options.max_num_matches), 65536 if aggressive else 32768)
        return options

    def _verification_options(self) -> pycolmap.TwoViewGeometryOptions:
        options = pycolmap.TwoViewGeometryOptions()
        options.multiple_models = False
        options.ransac.random_seed = self.config.reconstruction_random_seed
        return options

    def _mapping_options(self, aggressive: bool) -> pycolmap.IncrementalPipelineOptions:
        options = pycolmap.IncrementalPipelineOptions()
        options.multiple_models = False
        options.max_num_models = 1
        options.extract_colors = True
        options.random_seed = self.config.reconstruction_random_seed
        options.init_num_trials = max(400, options.init_num_trials)
        options.min_model_size = min(int(options.min_model_size), 6)
        options.mapper.random_seed = self.config.reconstruction_random_seed
        options.mapper.init_max_reg_trials = max(4, options.mapper.init_max_reg_trials)
        if aggressive:
            options.init_num_trials = max(1000, int(options.init_num_trials))
            options.min_num_matches = min(int(options.min_num_matches), 10)
            options.min_model_size = min(int(options.min_model_size), 4)
            options.mapper.init_min_num_inliers = min(int(options.mapper.init_min_num_inliers), 60)
            options.mapper.abs_pose_min_num_inliers = min(int(options.mapper.abs_pose_min_num_inliers), 24)
            options.mapper.abs_pose_min_inlier_ratio = min(
                float(options.mapper.abs_pose_min_inlier_ratio),
                0.18,
            )
            options.mapper.filter_max_reproj_error = max(
                float(options.mapper.filter_max_reproj_error),
                8.0,
            )
            options.triangulation.ignore_two_view_tracks = False
            options.triangulation.complete_max_transitivity = max(
                int(options.triangulation.complete_max_transitivity),
                8,
            )
        return options

    def _is_healthy_reconstruction(self, registered_keyframes: int, total_keyframes: int) -> bool:
        if total_keyframes <= 0:
            return False
        ratio = registered_keyframes / total_keyframes
        minimum_keyframes = min(total_keyframes, self.config.reconstruction_min_registered_keyframes)
        return ratio >= self.config.reconstruction_min_registered_ratio or registered_keyframes >= minimum_keyframes

    def _run_attempt(
        self,
        keyframes: list[Keyframe],
        workspace_dir: Path,
        *,
        attempt_config: AttemptConfig,
    ) -> ReconstructionAttempt:
        images_dir = workspace_dir / "images"
        attempt_dir = workspace_dir / f"colmap_{attempt_config.label}"
        database_path = attempt_dir / "database.db"
        sparse_dir = attempt_dir / "sparse"
        sparse_dir.mkdir(parents=True, exist_ok=True)

        image_names = [keyframe.image_name for keyframe in keyframes]
        reader_options = pycolmap.ImageReaderOptions()
        reader_options.camera_model = attempt_config.camera_model
        reader_options.default_focal_length_factor = 1.2
        camera_params = self._camera_params(attempt_config.camera_model) if attempt_config.use_camera_prior else ""
        if camera_params:
            reader_options.camera_params = camera_params

        extraction_options = self._feature_extraction_options(attempt_config.aggressive)
        matching_options = self._matching_options(attempt_config.aggressive)
        verification_options = self._verification_options()
        mapping_options = self._mapping_options(attempt_config.aggressive)

        try:
            pycolmap.extract_features(
                database_path=str(database_path),
                image_path=str(images_dir),
                image_names=image_names,
                camera_mode=pycolmap.CameraMode.SINGLE,
                reader_options=reader_options,
                extraction_options=extraction_options,
                device=pycolmap.Device.auto,
            )
            if attempt_config.matcher == "sequential":
                pairing_options = pycolmap.SequentialPairingOptions()
                pairing_options.overlap = max(1, min(max(8, len(keyframes) // 2), len(keyframes) - 1))
                pairing_options.quadratic_overlap = True
                pairing_options.loop_detection = False
                pycolmap.match_sequential(
                    database_path=str(database_path),
                    matching_options=matching_options,
                    pairing_options=pairing_options,
                    verification_options=verification_options,
                    device=pycolmap.Device.auto,
                )
            else:
                pairing_options = pycolmap.ExhaustivePairingOptions()
                pycolmap.match_exhaustive(
                    database_path=str(database_path),
                    matching_options=matching_options,
                    pairing_options=pairing_options,
                    verification_options=verification_options,
                    device=pycolmap.Device.auto,
                )

            reconstructions = pycolmap.incremental_mapping(
                database_path=str(database_path),
                image_path=str(images_dir),
                output_path=str(sparse_dir),
                options=mapping_options,
            )
            if not reconstructions:
                return ReconstructionAttempt(
                    label=attempt_config.label,
                    reconstruction=None,
                    registered_names=set(),
                    summary=f"{attempt_config.label}:0/{len(keyframes)}",
                    camera_model=attempt_config.camera_model,
                    error="no_valid_reconstruction",
                )

            reconstruction = max(
                reconstructions.values(),
                key=lambda item: (item.num_reg_images(), item.num_points3D()),
            )
            registered_names = {
                reconstruction.image(image_id).name
                for image_id in reconstruction.reg_image_ids()
            }
            return ReconstructionAttempt(
                label=attempt_config.label,
                reconstruction=reconstruction,
                registered_names=registered_names,
                summary=(
                    f"{attempt_config.label}:{len(registered_names)}/{len(keyframes)}"
                    f":pts={reconstruction.num_points3D()}"
                ),
                camera_model=attempt_config.camera_model,
            )
        except Exception as exc:  # noqa: BLE001
            return ReconstructionAttempt(
                label=attempt_config.label,
                reconstruction=None,
                registered_names=set(),
                summary=f"{attempt_config.label}:0/{len(keyframes)}",
                camera_model=attempt_config.camera_model,
                error=str(exc),
            )

    def reconstruct(
        self,
        keyframes: list[Keyframe],
        workspace_dir: Path,
    ) -> GeometryResult:
        if len(keyframes) < 2:
            raise RuntimeError("At least two keyframes are required for reconstruction.")

        images_dir = workspace_dir / "images"
        if not images_dir.exists():
            raise RuntimeError("Expected extracted keyframes under workspace/images.")

        attempts: list[ReconstructionAttempt] = []
        attempt_configs: list[AttemptConfig] = []
        if self._camera_prior_available():
            attempt_configs.extend(
                [
                    AttemptConfig(
                        label="video_sequential_prior",
                        matcher="sequential",
                        camera_model="PINHOLE",
                        use_camera_prior=True,
                    ),
                    AttemptConfig(
                        label="video_exhaustive_prior",
                        matcher="exhaustive",
                        camera_model="PINHOLE",
                        use_camera_prior=True,
                    ),
                ]
            )
        attempt_configs.extend(
            [
                AttemptConfig(
                    label="video_sequential",
                    matcher="sequential",
                    camera_model="SIMPLE_RADIAL",
                ),
                AttemptConfig(
                    label="video_exhaustive_dense",
                    matcher="exhaustive",
                    camera_model="SIMPLE_RADIAL",
                    aggressive=True,
                ),
            ]
        )

        best_successful_attempt: ReconstructionAttempt | None = None
        reconstruction_attempt: ReconstructionAttempt | None = None
        for attempt_config in attempt_configs:
            attempt = self._run_attempt(
                keyframes,
                workspace_dir,
                attempt_config=attempt_config,
            )
            attempts.append(attempt)
            if attempt.reconstruction is None:
                continue
            if (
                best_successful_attempt is None
                or (
                    len(attempt.registered_names),
                    attempt.reconstruction.num_points3D(),
                ) > (
                    len(best_successful_attempt.registered_names),
                    best_successful_attempt.reconstruction.num_points3D(),
                )
            ):
                best_successful_attempt = attempt
            if self._is_healthy_reconstruction(len(attempt.registered_names), len(keyframes)):
                reconstruction_attempt = attempt
                if attempt_config.matcher == "exhaustive":
                    break

        if reconstruction_attempt is None:
            successful_attempts = [attempt for attempt in attempts if attempt.reconstruction is not None]
            if not successful_attempts:
                details = "; ".join(
                    f"{attempt.summary} ({attempt.error})" if attempt.error else attempt.summary
                    for attempt in attempts
                )
                raise RuntimeError(f"pycolmap did not produce a valid reconstruction. Attempts: {details}")
            reconstruction_attempt = best_successful_attempt
            assert reconstruction_attempt is not None
        registered_keyframes = len(reconstruction_attempt.registered_names)
        if not self._is_healthy_reconstruction(registered_keyframes, len(keyframes)):
            details = "; ".join(
                f"{attempt.summary} ({attempt.error})" if attempt.error else attempt.summary
                for attempt in attempts
            )
            raise RuntimeError(
                "Reconstruction registered too few keyframes "
                f"({registered_keyframes}/{len(keyframes)}). Attempts: {details}"
            )

        reconstruction = reconstruction_attempt.reconstruction

        raw_points = np.asarray([reconstruction.point3D(pid).xyz for pid in reconstruction.point3D_ids()], dtype=float)
        camera_centers = np.asarray(
            [reconstruction.image(image_id).projection_center() for image_id in reconstruction.reg_image_ids()],
            dtype=float,
        )
        centroid, rotation = _pca_transform(camera_centers, raw_points)
        transform_point = _transformer(centroid, rotation)

        serialized_points: list[ReconstructionPoint] = []
        point_lookup: dict[int, np.ndarray] = {}
        point_ids = sorted(int(point_id) for point_id in reconstruction.point3D_ids())
        max_points = max(1000, int(self.config.reconstruction_max_serialized_points))
        stride = max(1, len(point_ids) // max_points)
        for point_id in point_ids[::stride]:
            point = reconstruction.point3D(point_id)
            xyz = transform_point(np.asarray(point.xyz, dtype=float))
            point_lookup[int(point_id)] = xyz
            serialized_points.append(
                ReconstructionPoint(
                    point_id=int(point_id),
                    xyz=_as_list(xyz),
                    color=[int(channel) for channel in np.asarray(point.color, dtype=int).reshape(-1).tolist()],
                    error=float(point.error) if point.has_error() else None,
                )
            )

        for point_id in point_ids:
            if int(point_id) not in point_lookup:
                point_lookup[int(point_id)] = transform_point(np.asarray(reconstruction.point3D(point_id).xyz, dtype=float))

        serialized_cameras: list[CameraPose] = []
        registered = reconstruction_attempt.registered_names
        for keyframe in keyframes:
            keyframe.registered = keyframe.image_name in registered

        for image_id in reconstruction.reg_image_ids():
            image = reconstruction.image(image_id)
            camera = reconstruction.camera(image.camera_id)
            pose = image.cam_from_world()
            center = transform_point(np.asarray(image.projection_center(), dtype=float))
            view_direction = np.asarray(image.viewing_direction(), dtype=float) @ rotation
            serialized_cameras.append(
                CameraPose(
                    keyframe_id=next(
                        frame.keyframe_id for frame in keyframes if frame.image_name == image.name
                    ),
                    image_name=image.name,
                    camera_id=int(image.camera_id),
                    center=_as_list(center),
                    view_direction=_as_list(view_direction),
                    cam_from_world=np.asarray(pose.matrix(), dtype=float).tolist(),
                    intrinsics={
                        "fx": float(camera.focal_length_x),
                        "fy": float(camera.focal_length_y),
                        "cx": float(camera.principal_point_x),
                        "cy": float(camera.principal_point_y),
                        "width": float(camera.width),
                        "height": float(camera.height),
                    },
                )
            )

        if serialized_points:
            point_xyz = np.asarray([point.xyz for point in serialized_points], dtype=float)
            min_corner = point_xyz.min(axis=0)
            max_corner = point_xyz.max(axis=0)
        elif serialized_cameras:
            camera_xyz = np.asarray([camera.center for camera in serialized_cameras], dtype=float)
            min_corner = camera_xyz.min(axis=0)
            max_corner = camera_xyz.max(axis=0)
        else:
            min_corner = np.zeros(3)
            max_corner = np.ones(3)
        scene_diagonal = float(np.linalg.norm(max_corner - min_corner))

        ply_path = workspace_dir / "reconstruction_sparse.ply"
        reconstruction.export_PLY(str(ply_path))
        registered_ratio = registered_keyframes / max(len(keyframes), 1)

        first_camera = serialized_cameras[0].intrinsics if serialized_cameras else {
            "fx": 0.0,
            "fy": 0.0,
            "cx": 0.0,
            "cy": 0.0,
            "width": 0.0,
            "height": 0.0,
        }
        artifact = ReconstructionArtifact(
            frame_id="reconstruction_world",
            scale_status="unknown_or_relative",
            alignment_method="pca_aligned_reconstruction",
            camera_model=reconstruction_attempt.camera_model,
            intrinsics=first_camera,
            keyframes=keyframes,
            cameras=serialized_cameras,
            sparse_points=serialized_points,
            sparse_ply_path=str(ply_path),
            scene_extent={"min": _as_list(min_corner), "max": _as_list(max_corner)},
        )

        return GeometryResult(
            reconstruction=reconstruction,
            artifact=artifact,
            point_lookup=point_lookup,
            transform_point=transform_point,
            scene_diagonal=max(scene_diagonal, 1e-6),
            registered_keyframes=registered_keyframes,
            registered_keyframe_ratio=registered_ratio,
            reconstruction_strategy=reconstruction_attempt.label,
            attempt_summaries=[attempt.summary for attempt in attempts],
        )
