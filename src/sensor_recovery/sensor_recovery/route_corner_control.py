"""ROS-free geometry for corner-safe cmd_vel route following."""

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from sensor_recovery.path_follow_control import normalize_angle


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class RouteGeometry:
    """Precomputed path lengths and hard-corner indices."""

    points: Tuple[Point2D, ...]
    cumulative_m: Tuple[float, ...]
    corner_indices: Tuple[int, ...]


def cumulative_lengths(points: Sequence[Point2D]) -> Tuple[float, ...]:
    """Return cumulative polyline distance at every point."""
    if len(points) < 2:
        raise ValueError("route must contain at least two points")
    result = [0.0]
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        if not math.isfinite(distance):
            raise ValueError("route coordinates must be finite")
        result.append(result[-1] + distance)
    if result[-1] <= 0.0:
        raise ValueError("route must span a non-zero distance")
    return tuple(result)


def _sample_index(
    cumulative_m: Sequence[float], index: int, distance_m: float, direction: int
) -> int:
    target = cumulative_m[index] + direction * distance_m
    result = index
    while 0 <= result + direction < len(cumulative_m):
        result += direction
        if direction < 0 and cumulative_m[result] <= target:
            break
        if direction > 0 and cumulative_m[result] >= target:
            break
    return result


def detect_corner_indices(
    points: Sequence[Point2D],
    angle_threshold_rad: float,
    sample_distance_m: float,
    cluster_distance_m: float,
) -> Tuple[int, ...]:
    """Detect and cluster meaningful turns using path-distance-sized arms."""
    cumulative = cumulative_lengths(points)
    if angle_threshold_rad <= 0.0 or sample_distance_m <= 0.0:
        raise ValueError("corner thresholds must be positive")
    candidates = []
    for index in range(1, len(points) - 1):
        before = _sample_index(cumulative, index, sample_distance_m, -1)
        after = _sample_index(cumulative, index, sample_distance_m, 1)
        if before == index or after == index:
            continue
        incoming = math.atan2(
            points[index][1] - points[before][1],
            points[index][0] - points[before][0],
        )
        outgoing = math.atan2(
            points[after][1] - points[index][1],
            points[after][0] - points[index][0],
        )
        angle = abs(normalize_angle(outgoing - incoming))
        if angle >= angle_threshold_rad:
            candidates.append((index, angle))

    if not candidates:
        return ()
    clusters = [[candidates[0]]]
    for candidate in candidates[1:]:
        previous_index = clusters[-1][-1][0]
        if cumulative[candidate[0]] - cumulative[previous_index] <= cluster_distance_m:
            clusters[-1].append(candidate)
        else:
            clusters.append([candidate])
    return tuple(max(cluster, key=lambda item: item[1])[0] for cluster in clusters)


def build_route_geometry(
    points: Sequence[Point2D],
    angle_threshold_rad: float,
    sample_distance_m: float,
    cluster_distance_m: float,
) -> RouteGeometry:
    """Validate a route and precompute its corner geometry."""
    normalized = tuple((float(point[0]), float(point[1])) for point in points)
    cumulative = cumulative_lengths(normalized)
    corners = detect_corner_indices(
        normalized,
        angle_threshold_rad,
        sample_distance_m,
        cluster_distance_m,
    )
    return RouteGeometry(normalized, cumulative, corners)


def interior_corner_indices(
    geometry: RouteGeometry, start_guard_m: float, end_guard_m: float
) -> Tuple[int, ...]:
    """Remove planner artifacts immediately after start and before goal."""
    total = geometry.cumulative_m[-1]
    return tuple(
        index
        for index in geometry.corner_indices
        if geometry.cumulative_m[index] >= max(0.0, start_guard_m)
        and total - geometry.cumulative_m[index] >= max(0.0, end_guard_m)
    )


def select_target_before_index(
    geometry: RouteGeometry,
    closest_index: int,
    lookahead_m: float,
    stop_index: int,
) -> Tuple[Point2D, int]:
    """Select a lookahead target without crossing a hard corner."""
    closest = max(0, min(closest_index, len(geometry.points) - 1))
    stop = max(closest, min(stop_index, len(geometry.points) - 1))
    target_distance = geometry.cumulative_m[closest] + max(lookahead_m, 0.0)
    index = closest
    while index < stop and geometry.cumulative_m[index] < target_distance:
        index += 1
    return geometry.points[index], index


def heading_after_index(
    geometry: RouteGeometry, index: int, sample_distance_m: float
) -> float:
    """Return the route heading after an index using a stable distance arm."""
    index = max(0, min(index, len(geometry.points) - 2))
    after = _sample_index(
        geometry.cumulative_m, index, max(sample_distance_m, 1e-6), 1
    )
    start = geometry.points[index]
    end = geometry.points[after]
    return math.atan2(end[1] - start[1], end[0] - start[0])


def remaining_path_distance(
    geometry: RouteGeometry, closest_index: int, target_index: int
) -> float:
    """Return non-negative path distance between two progress indices."""
    closest = max(0, min(closest_index, len(geometry.points) - 1))
    target = max(closest, min(target_index, len(geometry.points) - 1))
    return geometry.cumulative_m[target] - geometry.cumulative_m[closest]


def corner_speed_limit(
    max_speed: float, remaining_m: float, slowdown_distance_m: float
) -> float:
    """Linearly reduce forward speed while approaching a hard corner."""
    if remaining_m <= 0.0:
        return 0.0
    if slowdown_distance_m <= 0.0 or remaining_m >= slowdown_distance_m:
        return max_speed
    return max_speed * remaining_m / slowdown_distance_m
