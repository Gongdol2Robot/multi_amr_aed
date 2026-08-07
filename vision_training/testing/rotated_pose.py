"""검출 박스를 세워서 관절을 얻고, 좌표를 원본으로 되돌린다.

사전학습 YOLO Pose 는 COCO 의 서 있는 사람으로 배워서 누운 대상을 거의 잡지
못한다. 목각인형(103x39 px)으로 잰 값이다.

    원본 그대로            검출 없음 (imgsz 640 / 960 / 1280 전부)
    모델을 키움 (n/s/m/l)  검출 없음
    90도 돌려서 검출       conf 0.875, 관절 17/17

그래서 박스를 잘라 세운 뒤 추론한다. 다만 세운 상태의 좌표를 그대로 쓰면
종횡비가 2.66 에서 0.40 으로 뒤집혀 "서 있는 사람"이 되어 버리므로,
회전 행렬의 역행렬로 원본 좌표계에 되돌린다. 회전은 어파인 변환이라 가역이고,
6배로 확대한 뒤 돌리니 보간 오차도 원본 픽셀의 1/6 로 줄어든다.

되돌린 관절은 pose_posture.classify_posture 에 그대로 넣을 수 있다. 판정
규칙은 그쪽 것을 쓰고, 여기서는 "관절을 어떻게든 얻는" 일만 한다.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from pose_posture import LEFT_HIP, LEFT_SHOULDER, RIGHT_HIP, RIGHT_SHOULDER


# 박스 주변을 얼마나 더 넣을지(px)와 확대 배율. 배율이 크면 역변환 오차가
# 줄지만 느려진다. 6배가 실측 균형점이었다.
BOX_PADDING = 30
UPSCALE = 6
TORSO_INDEXES = (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP)


def candidate_angles(width: float, height: float) -> tuple[int, ...]:
    """박스 모양으로 회전 각도의 순서를 정한다.

    가로로 길면 누웠을 가능성이 크니 90도부터 본다. 후보 자체는 줄이지 않아
    넘어지는 중처럼 애매한 구간에서 놓치지 않게 한다.
    """
    if width > height * 1.2:
        return (90, 270, 0)
    return (0, 90, 270)


def _to_original(points, matrix, offset_x, offset_y, crop_x, crop_y):
    """회전·확대·잘라내기를 역순으로 풀어 원본 프레임 좌표로 되돌린다."""
    inverse = cv2.invertAffineTransform(matrix)
    homogeneous = np.hstack([points, np.ones((len(points), 1))])
    restored = (inverse @ homogeneous.T).T
    restored[:, 0] = (restored[:, 0] - offset_x) / UPSCALE + crop_x
    restored[:, 1] = (restored[:, 1] - offset_y) / UPSCALE + crop_y
    return restored


def estimate(
    model,
    frame,
    box,
    *,
    conf: float = 0.25,
    imgsz: int = 960,
    device: str = "",
    torso_conf: float = 0.3,
    early_exit_conf: float = 0.6,
):
    """box(x1, y1, x2, y2) 안의 관절을 (17, 3) 로 돌려준다.

    반환은 (keypoints, person_conf, rotation_deg) 이고 못 찾으면 None 이다.
    keypoints 의 각 행은 (x, y, conf) 로 classify_posture 와 형식이 같다.
    """
    height, width = frame.shape[:2]
    x1 = int(max(0, box[0] - BOX_PADDING))
    y1 = int(max(0, box[1] - BOX_PADDING))
    x2 = int(min(width, box[2] + BOX_PADDING))
    y2 = int(min(height, box[3] + BOX_PADDING))
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None

    enlarged = cv2.resize(
        crop, None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_CUBIC
    )
    crop_h, crop_w = enlarged.shape[:2]
    # 돌려도 잘리지 않도록 대각선 길이의 정사각형 위에 올린다.
    side = int(math.hypot(crop_h, crop_w))
    offset_x, offset_y = (side - crop_w) // 2, (side - crop_h) // 2
    canvas = np.full((side, side, 3), 200, dtype=np.uint8)
    canvas[offset_y:offset_y + crop_h, offset_x:offset_x + crop_w] = enlarged

    options = {"imgsz": imgsz, "verbose": False}
    if device:
        options["device"] = device

    # 자세를 판정하려면 몸통 네 점이 필요하다. person 신뢰도가 높아도 그 네
    # 점이 없으면 쓸모없으므로 (몸통 확보, 신뢰도) 순으로 고른다. 신뢰도만
    # 보면 관절 3개짜리 후보에서 멈춰 버린다.
    best = None
    best_key = (False, 0.0)
    for angle in candidate_angles(box[2] - box[0], box[3] - box[1]):
        matrix = cv2.getRotationMatrix2D((side / 2.0, side / 2.0), angle, 1.0)
        rotated = cv2.warpAffine(
            canvas, matrix, (side, side), borderValue=(200, 200, 200)
        )
        result = model.predict(rotated, conf=conf, **options)[0]
        if result.boxes is None or len(result.boxes) == 0:
            continue
        index = int(np.argmax(result.boxes.conf.cpu().numpy()))
        person_conf = float(result.boxes.conf[index])
        points = result.keypoints.xy[index].cpu().numpy().astype(float)
        scores = result.keypoints.conf[index].cpu().numpy().astype(float)
        has_torso = min(scores[i] for i in TORSO_INDEXES) >= torso_conf
        key = (has_torso, person_conf)
        if key <= best_key:
            continue
        best_key = key
        restored = _to_original(
            points, matrix, offset_x, offset_y, x1, y1
        )
        best = (
            np.column_stack([restored, scores]),
            person_conf,
            angle,
        )
        if has_torso and person_conf >= early_exit_conf:
            break

    return best
