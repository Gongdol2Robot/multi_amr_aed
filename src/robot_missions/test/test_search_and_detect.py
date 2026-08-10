import math

import pytest
from geometry_msgs.msg import PointStamped

from robot_missions.search_and_detect_node import (
    ApproachPoseCalculator,
    BoustrophedonPathPlanner,
    DetectionStateController,
    Point2D,
)


def test_narrow_polygon_generates_multiple_lanes_with_30_cm_spacing() -> None:
    planner = BoustrophedonPathPlanner(lane_spacing_m=0.30)
    polygon = [
        Point2D(0.0, 0.0),
        Point2D(2.0, 0.0),
        Point2D(2.0, 0.6),
        Point2D(0.0, 0.6),
    ]

    path = planner.generate(polygon)

    assert len(path) == 4
    expected = [
        (0.0, 0.15),
        (2.0, 0.15),
        (2.0, 0.45),
        (0.0, 0.45),
    ]
    for point, coordinates in zip(path, expected):
        assert (point.x, point.y) == pytest.approx(coordinates)


def test_reversing_coverage_starts_at_previous_end() -> None:
    planner = BoustrophedonPathPlanner(lane_spacing_m=0.30)
    polygon = [
        Point2D(0.0, 0.0),
        Point2D(2.0, 0.0),
        Point2D(2.0, 0.6),
        Point2D(0.0, 0.6),
    ]
    first_pass = planner.generate(polygon)

    second_pass = list(reversed(first_pass))

    assert second_pass[0] == first_pass[-1]
    assert second_pass[-1] == first_pass[0]


def test_polygon_contains_rejects_distant_detection() -> None:
    polygon = [
        Point2D(1.0, 1.0),
        Point2D(3.0, 1.0),
        Point2D(3.0, 2.0),
        Point2D(1.0, 2.0),
    ]

    assert BoustrophedonPathPlanner.contains(polygon, Point2D(2.0, 1.5))
    assert BoustrophedonPathPlanner.contains(
        polygon, Point2D(3.1, 1.5), tolerance_m=0.15
    )
    assert not BoustrophedonPathPlanner.contains(
        polygon, Point2D(0.0, 0.0), tolerance_m=0.15
    )


def test_nan_detection_confidence_is_rejected() -> None:
    controller = DetectionStateController(
        confidence_threshold=0.6,
        required_hits=1,
        reset_timeout_s=1.0,
    )
    location = PointStamped()
    location.header.frame_id = "map"
    location.point.x = 2.0
    location.point.y = 1.0

    assert not controller.register(location, math.nan, now_s=10.0)
    assert controller.target is None


def test_non_finite_approach_target_is_rejected() -> None:
    with pytest.raises(ValueError, match="finite"):
        ApproachPoseCalculator.calculate(
            Point2D(0.0, 0.0),
            Point2D(math.inf, 1.0),
            standoff_m=0.7,
        )
