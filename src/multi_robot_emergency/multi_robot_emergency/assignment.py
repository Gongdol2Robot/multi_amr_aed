"""Pure policies used by the emergency mission manager."""

from __future__ import annotations

import math
from collections.abc import Iterable

Position = tuple[float, float]


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the [-pi, pi] interval."""
    return math.atan2(math.sin(angle), math.cos(angle))


def path_length(points: Iterable[Position]) -> float:
    """Return the accumulated planar length of a path."""
    point_list = list(points)
    if not all(
        math.isfinite(value) for point in point_list for value in point
    ):
        raise ValueError("path points must contain only finite values")
    return sum(
        math.hypot(
            current[0] - previous[0], current[1] - previous[1]
        )
        for previous, current in zip(point_list, point_list[1:])
    )


def simplify_path(
    points: Iterable[Position], tolerance: float
) -> list[Position]:
    """Reduce grid-scale path jitter with Ramer-Douglas-Peucker."""
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError(
            "simplification_tolerance must be finite and non-negative"
        )
    point_list = list(points)
    path_length(point_list)
    if len(point_list) <= 2 or tolerance == 0.0:
        return point_list

    start = point_list[0]
    end = point_list[-1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    segment_length = math.hypot(dx, dy)
    if segment_length <= 1e-12:
        distances = [
            math.hypot(point[0] - start[0], point[1] - start[1])
            for point in point_list[1:-1]
        ]
    else:
        distances = [
            abs(dy * (point[0] - start[0]) - dx * (point[1] - start[1]))
            / segment_length
            for point in point_list[1:-1]
        ]
    if not distances:
        return [start, end]
    maximum = max(distances)
    if maximum <= tolerance:
        return [start, end]
    split = distances.index(maximum) + 1
    left = simplify_path(point_list[: split + 1], tolerance)
    right = simplify_path(point_list[split:], tolerance)
    return left[:-1] + right


def path_motion_cost(
    points: Iterable[Position],
    *,
    linear_speed: float,
    angular_speed: float,
    slowdown_turn_threshold: float,
    slowdown_penalty: float,
    simplification_tolerance: float = 0.0,
    initial_yaw: float | None = None,
    final_yaw: float | None = None,
) -> tuple[float, float, int]:
    """Estimate travel time from distance, turns, and corner slowdowns.

    Returns ``(estimated_seconds, total_turn_radians, slowdown_count)``.
    """
    if not math.isfinite(linear_speed) or linear_speed <= 0.0:
        raise ValueError("linear_speed must be positive")
    if not math.isfinite(angular_speed) or angular_speed <= 0.0:
        raise ValueError("angular_speed must be positive")
    if (
        not math.isfinite(slowdown_turn_threshold)
        or slowdown_turn_threshold < 0.0
    ):
        raise ValueError("slowdown_turn_threshold must be non-negative")
    if not math.isfinite(slowdown_penalty) or slowdown_penalty < 0.0:
        raise ValueError("slowdown_penalty must be non-negative")
    if initial_yaw is not None and not math.isfinite(initial_yaw):
        raise ValueError("initial_yaw must be finite")
    if final_yaw is not None and not math.isfinite(final_yaw):
        raise ValueError("final_yaw must be finite")

    point_list = list(points)
    distance = path_length(point_list)
    simplified_points = simplify_path(point_list, simplification_tolerance)
    headings: list[float] = []
    for previous, current in zip(
        simplified_points, simplified_points[1:]
    ):
        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
        if math.hypot(dx, dy) <= 1e-9:
            continue
        headings.append(math.atan2(dy, dx))

    internal_turn_angles = [
        abs(normalize_angle(current - previous))
        for previous, current in zip(headings, headings[1:])
    ]
    turn_angles = list(internal_turn_angles)
    if headings and initial_yaw is not None:
        turn_angles.insert(
            0, abs(normalize_angle(headings[0] - initial_yaw))
        )
    if headings and final_yaw is not None:
        turn_angles.append(abs(normalize_angle(final_yaw - headings[-1])))
    if not headings and initial_yaw is not None and final_yaw is not None:
        turn_angles.append(abs(normalize_angle(final_yaw - initial_yaw)))
    total_turn = sum(turn_angles)
    slowdown_count = sum(
        1
        for angle in internal_turn_angles
        if angle >= slowdown_turn_threshold
    )
    estimated_seconds = (
        distance / linear_speed
        + total_turn / angular_speed
        + slowdown_count * slowdown_penalty
    )
    return estimated_seconds, total_turn, slowdown_count
