"""YOLO Pose의 COCO 관절과 bbox로 사람 자세를 판정한다."""

from __future__ import annotations

from math import atan2, degrees


LEFT_SHOULDER, RIGHT_SHOULDER = 5, 6
LEFT_HIP, RIGHT_HIP = 11, 12
LEFT_KNEE, RIGHT_KNEE = 13, 14


def _center(keypoints, first: int, second: int, threshold: float):
    points = [keypoints[index] for index in (first, second)]
    visible = [point for point in points if point[2] >= threshold]
    if not visible:
        return None
    return (
        sum(point[0] for point in visible) / len(visible),
        sum(point[1] for point in visible) / len(visible),
    )


def classify_posture(
    keypoints,
    box,
    keypoint_conf: float = 0.25,
) -> tuple[str, dict[str, float]]:
    """17관절과 xyxy bbox를 자세 및 판단 지표로 변환한다."""
    x1, y1, x2, y2 = (float(value) for value in box)
    width = max(x2 - x1, 1.0)
    height = max(y2 - y1, 1.0)
    aspect_ratio = width / height
    metrics = {"aspect_ratio": aspect_ratio, "torso_angle": -1.0}

    shoulders = _center(
        keypoints, LEFT_SHOULDER, RIGHT_SHOULDER, keypoint_conf
    )
    hips = _center(keypoints, LEFT_HIP, RIGHT_HIP, keypoint_conf)
    knees = _center(keypoints, LEFT_KNEE, RIGHT_KNEE, keypoint_conf)
    if shoulders is None or hips is None:
        return ("FALLEN" if aspect_ratio >= 1.6 else "UNKNOWN"), metrics

    dx = abs(hips[0] - shoulders[0])
    dy = abs(hips[1] - shoulders[1])
    torso_angle = degrees(atan2(dy, dx))
    metrics["torso_angle"] = torso_angle

    if aspect_ratio >= 1.2 and torso_angle <= 40.0:
        return "FALLEN", metrics
    if aspect_ratio >= 1.6:
        return "FALLEN", metrics
    if knees is not None:
        hip_to_knee_x = abs(knees[0] - hips[0])
        hip_to_knee_y = abs(knees[1] - hips[1])
        if hip_to_knee_x > hip_to_knee_y * 0.8:
            return "SITTING", metrics
    return "STANDING", metrics
