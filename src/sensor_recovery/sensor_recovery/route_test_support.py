"""ROS-free helpers for controlled fallback route-following tests."""

import math
from typing import List, Sequence, Tuple


Point2D = Tuple[float, float]


def parse_flat_route(values: Sequence[float]) -> List[Point2D]:
    """Convert ``[x0, y0, x1, y1, ...]`` into validated map points."""
    if len(values) < 4 or len(values) % 2:
        raise ValueError("route must contain at least two complete x/y pairs")
    numbers = [float(value) for value in values]
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError("route coordinates must all be finite")
    return list(zip(numbers[0::2], numbers[1::2]))


def densify_route(points: Sequence[Point2D], spacing_m: float) -> List[Point2D]:
    """Sample a polyline densely enough for stable closest/lookahead progress."""
    if len(points) < 2:
        raise ValueError("route must contain at least two points")
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be positive")

    result = [points[0]]
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        if distance == 0.0:
            continue
        steps = max(1, math.ceil(distance / spacing_m))
        for index in range(1, steps + 1):
            ratio = index / steps
            result.append((start[0] + dx * ratio, start[1] + dy * ratio))
    if len(result) < 2:
        raise ValueError("route must span a non-zero distance")
    return result


def minimum_range_in_sector(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    half_angle_rad: float,
) -> float:
    """Return the nearest finite positive scan range in a forward sector."""
    if angle_increment == 0.0 or half_angle_rad <= 0.0:
        return math.inf
    candidates = []
    for index, value in enumerate(ranges):
        angle = angle_min + index * angle_increment
        if abs(angle) <= half_angle_rad and math.isfinite(value) and value > 0.0:
            candidates.append(float(value))
    return min(candidates, default=math.inf)
