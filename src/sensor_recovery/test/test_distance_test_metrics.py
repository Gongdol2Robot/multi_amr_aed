import math

from sensor_recovery.distance_test_metrics import (
    calculate_distance_test_metrics,
    expected_pose_after_forward,
    project_odom_pose_to_map,
)


def test_expected_pose_after_forward_uses_start_heading():
    expected = expected_pose_after_forward((1.0, 2.0, math.pi / 2), 0.5)
    assert math.isclose(expected[0], 1.0, abs_tol=1e-9)
    assert math.isclose(expected[1], 2.5, abs_tol=1e-9)
    assert math.isclose(expected[2], math.pi / 2, abs_tol=1e-9)


def test_perfect_motion_has_zero_error():
    result = calculate_distance_test_metrics(
        (0.0, 0.0, 0.0), (0.5, 0.0, 0.0), 0.5
    )
    assert math.isclose(result.forward_error_m, 0.0, abs_tol=1e-9)
    assert math.isclose(result.lateral_error_m, 0.0, abs_tol=1e-9)
    assert math.isclose(result.position_error_m, 0.0, abs_tol=1e-9)


def test_errors_are_expressed_in_start_frame():
    result = calculate_distance_test_metrics(
        (1.0, 1.0, math.pi / 2),
        (0.98, 1.47, math.radians(95.0)),
        0.5,
    )
    assert math.isclose(result.actual_forward_m, 0.47, abs_tol=1e-9)
    assert math.isclose(result.actual_lateral_m, 0.02, abs_tol=1e-9)
    assert math.isclose(result.forward_error_m, -0.03, abs_tol=1e-9)
    assert math.isclose(result.lateral_error_m, 0.02, abs_tol=1e-9)
    assert math.isclose(result.yaw_error_deg, 5.0, abs_tol=1e-9)


def test_position_error_combines_forward_and_lateral_error():
    result = calculate_distance_test_metrics(
        (0.0, 0.0, 0.0), (0.46, -0.03, 0.0), 0.5
    )
    assert math.isclose(result.position_error_m, 0.05, abs_tol=1e-9)


def test_project_odom_pose_to_rotated_map_frame():
    result = project_odom_pose_to_map(
        (0.8, 0.2, math.pi / 2),
        (2.0, 3.0, 0.0),
        (2.5, 3.0, math.radians(5.0)),
    )
    assert math.isclose(result[0], 0.8, abs_tol=1e-9)
    assert math.isclose(result[1], 0.7, abs_tol=1e-9)
    assert math.isclose(result[2], math.radians(95.0), abs_tol=1e-9)
