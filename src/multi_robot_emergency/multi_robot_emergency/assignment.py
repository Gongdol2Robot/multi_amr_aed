"""Pure policies used by the emergency mission manager."""

from __future__ import annotations

import math
from collections.abc import Iterable
Position = tuple[float, float]


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
