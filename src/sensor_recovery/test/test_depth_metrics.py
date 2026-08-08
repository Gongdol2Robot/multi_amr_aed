import numpy as np

from sensor_recovery.depth_metrics import (
    compute_depth_region_metrics,
    format_distance,
)


def test_depth_region_metrics_match_total_roi_ratios():
    depth = np.array(
        [
            [0, 100, 400, 600],
            [0, 200, 500, 800],
        ],
        dtype=np.uint16,
    )
    metrics = compute_depth_region_metrics(depth, (0, 0, 4, 2), 0.5)

    assert metrics.valid_ratio == 0.75
    assert metrics.close_ratio == 0.375
    assert metrics.minimum_m == 0.1
    assert metrics.median_m == 0.45
    assert metrics.close_median_m == 0.2


def test_depth_region_metrics_handle_no_valid_pixels():
    depth = np.zeros((4, 4), dtype=np.uint16)
    metrics = compute_depth_region_metrics(depth, (0, 0, 4, 4), 0.5)

    assert metrics.valid_ratio == 0.0
    assert metrics.close_ratio == 0.0
    assert metrics.minimum_m is None
    assert format_distance(metrics.p05_m) == "n/a"
