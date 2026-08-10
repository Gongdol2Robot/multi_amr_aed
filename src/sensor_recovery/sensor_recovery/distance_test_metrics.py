"""Pure error calculations for the open-loop cmd_vel distance test."""

import math
from dataclasses import dataclass
from typing import Tuple

from sensor_recovery.path_follow_control import normalize_angle


Pose2D = Tuple[float, float, float]


@dataclass(frozen=True)
class DistanceTestMetrics:
    expected_pose: Pose2D
    actual_pose: Pose2D
    actual_forward_m: float
    actual_lateral_m: float
    forward_error_m: float
    lateral_error_m: float
    position_error_m: float
    yaw_error_deg: float


def expected_pose_after_forward(start_pose: Pose2D, distance_m: float) -> Pose2D:
    """Return the expected map pose after ideal travel along the start yaw."""
    x, y, yaw = start_pose
    return (
        x + distance_m * math.cos(yaw),
        y + distance_m * math.sin(yaw),
        yaw,
    )


def project_odom_pose_to_map(
    start_map_pose: Pose2D,
    start_odom_pose: Pose2D,
    actual_odom_pose: Pose2D,
) -> Pose2D:
    """Project an odom delta onto the map pose captured at the same start time."""
    odom_dx = actual_odom_pose[0] - start_odom_pose[0]
    odom_dy = actual_odom_pose[1] - start_odom_pose[1]
    frame_yaw = start_map_pose[2] - start_odom_pose[2]
    map_dx = math.cos(frame_yaw) * odom_dx - math.sin(frame_yaw) * odom_dy
    map_dy = math.sin(frame_yaw) * odom_dx + math.cos(frame_yaw) * odom_dy
    return (
        start_map_pose[0] + map_dx,
        start_map_pose[1] + map_dy,
        normalize_angle(
            start_map_pose[2]
            + normalize_angle(actual_odom_pose[2] - start_odom_pose[2])
        ),
    )


def calculate_distance_test_metrics(
    start_pose: Pose2D,
    actual_pose: Pose2D,
    commanded_distance_m: float,
) -> DistanceTestMetrics:
    """Compare actual motion with an ideal forward command in the start frame."""
    start_x, start_y, start_yaw = start_pose
    actual_x, actual_y, actual_yaw = actual_pose
    delta_x = actual_x - start_x
    delta_y = actual_y - start_y
    actual_forward = delta_x * math.cos(start_yaw) + delta_y * math.sin(start_yaw)
    actual_lateral = -delta_x * math.sin(start_yaw) + delta_y * math.cos(start_yaw)
    expected = expected_pose_after_forward(start_pose, commanded_distance_m)
    return DistanceTestMetrics(
        expected_pose=expected,
        actual_pose=actual_pose,
        actual_forward_m=actual_forward,
        actual_lateral_m=actual_lateral,
        forward_error_m=actual_forward - commanded_distance_m,
        lateral_error_m=actual_lateral,
        position_error_m=math.hypot(actual_x - expected[0], actual_y - expected[1]),
        yaw_error_deg=math.degrees(normalize_angle(actual_yaw - start_yaw)),
    )
