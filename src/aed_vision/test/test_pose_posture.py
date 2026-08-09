"""관절과 bbox 기반 자세 판정 테스트."""

import numpy as np

from aed_vision.pose_posture import classify_posture


def _keypoints() -> np.ndarray:
    points = np.zeros((17, 3), dtype=float)
    points[:, 2] = 1.0
    return points


def test_horizontal_torso_is_fallen() -> None:
    points = _keypoints()
    points[5, :2] = (10, 10)
    points[6, :2] = (10, 12)
    points[11, :2] = (40, 10)
    points[12, :2] = (40, 12)

    posture, metrics = classify_posture(points, (0, 0, 50, 30))

    assert posture == "FALLEN"
    assert metrics["torso_angle_deg"] < 5


def test_vertical_torso_is_standing() -> None:
    points = _keypoints()
    points[5, :2] = (20, 10)
    points[6, :2] = (24, 10)
    points[11, :2] = (20, 50)
    points[12, :2] = (24, 50)
    points[13, :2] = (20, 80)
    points[14, :2] = (24, 80)

    posture, _ = classify_posture(points, (0, 0, 40, 100))

    assert posture == "STANDING"


def test_missing_torso_uses_box_shape() -> None:
    points = np.zeros((17, 3), dtype=float)

    assert classify_posture(points, (0, 0, 100, 40))[0] == "FALLEN"
    assert classify_posture(points, (0, 0, 40, 100))[0] == "UNKNOWN"
