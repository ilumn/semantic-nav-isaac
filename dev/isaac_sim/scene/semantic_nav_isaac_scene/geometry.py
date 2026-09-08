"""Small dependency-free geometry helpers shared by composition and tests."""

from __future__ import annotations

import math
from typing import Iterable, Mapping, Sequence


def box_bounds(center: Sequence[float], size: Sequence[float]) -> tuple[tuple[float, float], ...]:
    """Return ``(min, max)`` bounds for each axis of an axis-aligned box."""

    if len(center) != 3 or len(size) != 3:
        raise ValueError("center and size must both contain three values")
    if any(float(length) <= 0.0 for length in size):
        raise ValueError("box dimensions must be positive")
    return tuple(
        (float(midpoint) - float(length) / 2.0, float(midpoint) + float(length) / 2.0)
        for midpoint, length in zip(center, size)
    )


def warehouse_interior_bounds(walls: Iterable[Mapping[str, Sequence[float]]]) -> tuple[float, float, float, float]:
    """Infer the free-space XY rectangle from east/west/north/south wall boxes."""

    indexed = {str(wall["name"]): wall for wall in walls}
    required = {"east", "west", "north", "south"}
    if set(indexed) != required:
        raise ValueError(f"walls must be named {sorted(required)}")
    east = box_bounds(indexed["east"]["center"], indexed["east"]["size"])[0][0]
    west = box_bounds(indexed["west"]["center"], indexed["west"]["size"])[0][1]
    north = box_bounds(indexed["north"]["center"], indexed["north"]["size"])[1][0]
    south = box_bounds(indexed["south"]["center"], indexed["south"]["size"])[1][1]
    if not (west < east and south < north):
        raise ValueError("wall boxes do not enclose a valid interior")
    return west, east, south, north


def pose_is_inside_xy(xyz: Sequence[float], bounds: Sequence[float], *, margin: float = 0.0) -> bool:
    """Return whether an XYZ position lies within XY *bounds* after a margin."""

    if len(xyz) != 3 or len(bounds) != 4:
        raise ValueError("xyz must contain 3 values and bounds must contain 4")
    west, east, south, north = map(float, bounds)
    x, y = float(xyz[0]), float(xyz[1])
    return west + margin <= x <= east - margin and south + margin <= y <= north - margin


def rpy_to_quaternion(rpy: Sequence[float]) -> tuple[float, float, float, float]:
    """Convert extrinsic roll/pitch/yaw radians to a normalized ``wxyz`` quaternion."""

    if len(rpy) != 3:
        raise ValueError("rpy must contain three values")
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return normalize_quaternion(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    )


def normalize_quaternion(quaternion: Sequence[float]) -> tuple[float, float, float, float]:
    if len(quaternion) != 4:
        raise ValueError("quaternion must contain four values")
    values = tuple(float(value) for value in quaternion)
    norm = math.sqrt(sum(value * value for value in values))
    if norm == 0.0:
        raise ValueError("zero quaternion cannot be normalized")
    return tuple(value / norm for value in values)


def quaternion_to_matrix(quaternion: Sequence[float]) -> tuple[tuple[float, float, float], ...]:
    """Convert a ``wxyz`` quaternion to a 3x3 row-major rotation matrix."""

    w, x, y, z = normalize_quaternion(quaternion)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> tuple[float, float, float]:
    """Rotate a three-vector by a ``wxyz`` quaternion."""

    if len(vector) != 3:
        raise ValueError("vector must contain three values")
    matrix = quaternion_to_matrix(quaternion)
    return tuple(
        sum(matrix[row][column] * float(vector[column]) for column in range(3))
        for row in range(3)
    )


def fixed_parent_relative_translation(
    child_relative_xyz: Sequence[float],
    child_relative_wxyz: Sequence[float],
    parent_to_child_xyz: Sequence[float],
) -> tuple[float, float, float]:
    """Shift relative child odometry to its fixed parent frame.

    ``child_relative_xyz`` is assumed to start at zero, as Isaac's odometry
    node does.  Therefore the fixed offset correction is ``R_rel*t - t``.
    """

    if len(child_relative_xyz) != 3 or len(parent_to_child_xyz) != 3:
        raise ValueError("translations must contain three values")
    rotated = rotate_vector(child_relative_wxyz, parent_to_child_xyz)
    return tuple(
        float(child_relative_xyz[index])
        - (rotated[index] - float(parent_to_child_xyz[index]))
        for index in range(3)
    )


def fixed_parent_local_linear_velocity(
    child_local_velocity: Sequence[float],
    local_angular_velocity: Sequence[float],
    parent_to_child_xyz: Sequence[float],
) -> tuple[float, float, float]:
    """Shift a fixed child's local linear velocity to its parent origin."""

    if any(len(value) != 3 for value in (child_local_velocity, local_angular_velocity, parent_to_child_xyz)):
        raise ValueError("velocities and translation must contain three values")
    wx, wy, wz = (float(value) for value in local_angular_velocity)
    tx, ty, tz = (float(value) for value in parent_to_child_xyz)
    angular_cross_offset = (wy * tz - wz * ty, wz * tx - wx * tz, wx * ty - wy * tx)
    return tuple(
        float(child_local_velocity[index]) - angular_cross_offset[index]
        for index in range(3)
    )


def horizontal_focal_length_mm(horizontal_aperture_mm: float, horizontal_fov_rad: float) -> float:
    """Return pinhole focal length for an aperture width and horizontal field of view."""

    aperture = float(horizontal_aperture_mm)
    fov = float(horizontal_fov_rad)
    if aperture <= 0.0:
        raise ValueError("horizontal aperture must be positive")
    if not 0.0 < fov < math.pi:
        raise ValueError("horizontal field of view must be between 0 and pi")
    return aperture / (2.0 * math.tan(fov / 2.0))


def compose_pose(
    parent_xyz: Sequence[float],
    parent_rpy: Sequence[float],
    child_xyz: Sequence[float],
) -> tuple[float, float, float]:
    """Rotate a child translation by parent RPY and add the parent translation."""

    rotated = rotate_vector(rpy_to_quaternion(parent_rpy), child_xyz)
    return tuple(float(parent_xyz[index]) + rotated[index] for index in range(3))
