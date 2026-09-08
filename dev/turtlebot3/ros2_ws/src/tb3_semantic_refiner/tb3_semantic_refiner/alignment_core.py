from __future__ import annotations

import math
from dataclasses import dataclass


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(slots=True)
class SimilarityTransform2D:
    scale: float
    yaw_rad: float
    tx: float
    ty: float

    def apply(self, x: float, y: float) -> tuple[float, float]:
        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)
        out_x = self.tx + self.scale * (cos_yaw * x - sin_yaw * y)
        out_y = self.ty + self.scale * (sin_yaw * x + cos_yaw * y)
        return out_x, out_y


@dataclass(slots=True)
class AlignmentEstimate:
    transform: SimilarityTransform2D
    correspondences: int
    mean_error: float
    rms_error: float
    max_error: float
    source_span: float
    target_span: float
    confidence: float
    accepted: bool


def estimate_similarity_transform_2d(
    source_points: list[tuple[float, float]],
    target_points: list[tuple[float, float]],
) -> SimilarityTransform2D:
    """Estimate a 2D similarity transform from point correspondences."""
    if len(source_points) != len(target_points) or len(source_points) < 2:
        raise ValueError("Need at least two matched source/target points.")

    sx = sum(x for x, _ in source_points) / len(source_points)
    sy = sum(y for _, y in source_points) / len(source_points)
    tx = sum(x for x, _ in target_points) / len(target_points)
    ty = sum(y for _, y in target_points) / len(target_points)

    centered_source = [(x - sx, y - sy) for x, y in source_points]
    centered_target = [(x - tx, y - ty) for x, y in target_points]

    num_real = 0.0
    num_imag = 0.0
    denom = 0.0
    for (xs, ys), (xt, yt) in zip(centered_source, centered_target):
        num_real += xt * xs + yt * ys
        num_imag += yt * xs - xt * ys
        denom += xs * xs + ys * ys
    if denom == 0.0:
        raise ValueError("Degenerate source points.")

    scale = math.hypot(num_real, num_imag) / denom
    yaw = math.atan2(num_imag, num_real)

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    trans_x = tx - scale * (cos_yaw * sx - sin_yaw * sy)
    trans_y = ty - scale * (sin_yaw * sx + cos_yaw * sy)
    return SimilarityTransform2D(scale=scale, yaw_rad=yaw, tx=trans_x, ty=trans_y)


def estimate_alignment_2d(
    source_points: list[tuple[float, float]],
    target_points: list[tuple[float, float]],
    max_rms_ratio: float = 0.25,
) -> AlignmentEstimate:
    transform = estimate_similarity_transform_2d(source_points, target_points)

    errors: list[float] = []
    sx = sum(x for x, _ in source_points) / len(source_points)
    sy = sum(y for _, y in source_points) / len(source_points)
    tx = sum(x for x, _ in target_points) / len(target_points)
    ty = sum(y for _, y in target_points) / len(target_points)

    for (source_x, source_y), (target_x, target_y) in zip(source_points, target_points):
        aligned_x, aligned_y = transform.apply(source_x, source_y)
        errors.append(math.hypot(aligned_x - target_x, aligned_y - target_y))

    source_span = max(math.hypot(x - sx, y - sy) for x, y in source_points)
    target_span = max(math.hypot(x - tx, y - ty) for x, y in target_points)
    mean_error = sum(errors) / len(errors)
    rms_error = math.sqrt(sum(error * error for error in errors) / len(errors))
    max_error = max(errors)
    span = max(source_span, target_span, 1e-6)
    normalized_rms = rms_error / span
    coverage = _clamp((len(source_points) - 1) / 5.0, 0.0, 1.0)
    residual_score = _clamp(1.0 - normalized_rms / max(max_rms_ratio, 1e-6), 0.0, 1.0)
    confidence = round(coverage * residual_score, 4)
    accepted = rms_error <= span * max_rms_ratio
    return AlignmentEstimate(
        transform=transform,
        correspondences=len(source_points),
        mean_error=mean_error,
        rms_error=rms_error,
        max_error=max_error,
        source_span=source_span,
        target_span=target_span,
        confidence=confidence,
        accepted=accepted,
    )
