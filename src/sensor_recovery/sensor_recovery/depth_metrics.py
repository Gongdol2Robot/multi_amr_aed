"""Diagnostic measurements for 16-bit millimetre depth images."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


Roi = Tuple[int, int, int, int]


@dataclass(frozen=True)
class DepthRegionMetrics:
    """Pixel ratios and robust distances for one depth-image region."""

    valid_ratio: float
    close_ratio: float
    minimum_m: Optional[float]
    p05_m: Optional[float]
    median_m: Optional[float]
    close_median_m: Optional[float]


def compute_depth_region_metrics(
    depth_image_mm: np.ndarray,
    roi: Roi,
    obstacle_distance_m: float,
) -> DepthRegionMetrics:
    """Measure the same ROI ratios used by the follower's safety decision."""
    x0, y0, x1, y1 = roi
    region = depth_image_mm[y0:y1, x0:x1]
    if region.size == 0:
        return DepthRegionMetrics(0.0, 0.0, None, None, None, None)

    valid_mask = (region > 0) & np.isfinite(region)
    valid_values = region[valid_mask].astype(np.float64)
    valid_ratio = float(np.count_nonzero(valid_mask) / region.size)
    if valid_values.size == 0:
        return DepthRegionMetrics(valid_ratio, 0.0, None, None, None, None)

    close_mask = valid_mask & (region < obstacle_distance_m * 1000.0)
    close_values = region[close_mask].astype(np.float64)
    close_ratio = float(np.count_nonzero(close_mask) / region.size)
    close_median_m = (
        None
        if close_values.size == 0
        else float(np.median(close_values) / 1000.0)
    )
    return DepthRegionMetrics(
        valid_ratio=valid_ratio,
        close_ratio=close_ratio,
        minimum_m=float(np.min(valid_values) / 1000.0),
        p05_m=float(np.percentile(valid_values, 5) / 1000.0),
        median_m=float(np.median(valid_values) / 1000.0),
        close_median_m=close_median_m,
    )


def format_distance(distance_m: Optional[float]) -> str:
    """Format an optional metric distance for logs and overlays."""
    return "n/a" if distance_m is None else f"{distance_m:.3f}m"
