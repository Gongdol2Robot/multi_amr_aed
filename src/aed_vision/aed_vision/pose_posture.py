"""COCO 17관절과 bbox로 실제 사람의 자세를 판정한다."""

from __future__ import annotations

from math import atan2, degrees


LEFT_SHOULDER, RIGHT_SHOULDER = 5, 6
LEFT_HIP, RIGHT_HIP = 11, 12
LEFT_KNEE, RIGHT_KNEE = 13, 14
TORSO_INDEXES = (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP)


def _center(keypoints, first: int, second: int, threshold: float):
    # 좌우 한 쌍(예: 양 어깨) 중 confidence가 threshold 이상인 점만 평균한다.
    # 한쪽이 가려져도(예: 옆으로 누움) 보이는 한 점만으로 중심을 낼 수 있게
    # 최소 1개만 있으면 계산하고, 둘 다 안 보이면 None을 반환한다.
    visible = [
        keypoints[index]
        for index in (first, second)
        if keypoints[index][2] >= threshold
    ]
    if not visible:
        return None
    return (
        sum(point[0] for point in visible) / len(visible),
        sum(point[1] for point in visible) / len(visible),
    )


def classify_posture(
    keypoints,
    box,
    keypoint_conf: float = 0.3,
) -> tuple[str, dict[str, float]]:
    """자세와 판단 근거를 반환한다.

    몸통이 수평이고 bbox가 가로로 길면 FALLEN, 엉덩이에서 무릎 방향이
    수평에 가까우면 SITTING, 그 외에는 STANDING으로 판정한다.
    """
    x1, y1, x2, y2 = (float(value) for value in box)
    width = max(x2 - x1, 1.0)
    height = max(y2 - y1, 1.0)
    # bbox가 세로보다 가로로 길수록 (aspect_ratio가 클수록) 누워 있을 가능성이 높다.
    aspect_ratio = width / height
    metrics = {"aspect_ratio": aspect_ratio, "torso_angle_deg": -1.0}

    shoulders = _center(
        keypoints, LEFT_SHOULDER, RIGHT_SHOULDER, keypoint_conf
    )
    hips = _center(keypoints, LEFT_HIP, RIGHT_HIP, keypoint_conf)
    knees = _center(keypoints, LEFT_KNEE, RIGHT_KNEE, keypoint_conf)
    # 어깨나 엉덩이가 전혀 안 보이면(가림·측면) 몸통 각도를 계산할 수 없다.
    # 이때는 bbox 형태만으로 판단하고, 애매하면 UNKNOWN으로 둔다(과잉 확정 방지).
    if shoulders is None or hips is None:
        return ("FALLEN" if aspect_ratio >= 1.6 else "UNKNOWN"), metrics

    # 어깨-엉덩이를 잇는 몸통 벡터가 수평(작은 각도)이면 누운 자세다.
    # atan2(dy, dx)를 쓰므로 각도는 0(완전 수평)~90도(완전 수직) 범위다.
    dx = abs(hips[0] - shoulders[0])
    dy = abs(hips[1] - shoulders[1])
    torso_angle = degrees(atan2(dy, dx))
    metrics["torso_angle_deg"] = torso_angle

    # 두 조건 중 하나만 만족해도 FALLEN: (1) 몸통이 눕고 bbox도 어느 정도 넓거나
    # (2) 몸통 각도를 신뢰하기 애매해도 bbox가 아주 넓게 누운 경우(측면·가림 대비).
    if aspect_ratio >= 1.2 and torso_angle <= 40.0:
        return "FALLEN", metrics
    if aspect_ratio >= 1.6:
        return "FALLEN", metrics
    # 무릎이 엉덩이보다 수평 방향으로 더 벌어져 있으면(가로 거리 > 세로 거리*0.8)
    # 다리를 앞으로 뻗은 착석 자세로 본다.
    if knees is not None:
        hip_to_knee_x = abs(knees[0] - hips[0])
        hip_to_knee_y = abs(knees[1] - hips[1])
        if hip_to_knee_x > hip_to_knee_y * 0.8:
            return "SITTING", metrics
    return "STANDING", metrics
