import math
from typing import List


def nearest_circular_ranges(
    source: list[float],
    angle_min: float,
    angle_increment: float,
    target_count: int,
) -> List[float]:
    if target_count <= 0:
        raise ValueError("target_count must be positive")
    if not source or angle_increment == 0.0:
        return [math.nan] * target_count

    target_increment = (2.0 * math.pi) / float(target_count)
    source_count = len(source)
    ranges: List[float] = []

    for i in range(target_count):
        angle = i * target_increment
        relative = (angle - float(angle_min)) % (2.0 * math.pi)
        source_index = int(round(relative / float(angle_increment))) % source_count
        ranges.append(source[source_index])

    return ranges
