"""목각인형 crop을 HOG 특징과 SVM으로 FALLEN/NON_FALLEN 분류한다.

흐름은 ``BGR crop -> 회색조/64x64 -> HOG 1,764개 숫자 -> SVM 0 또는 1``이다.
YOLO detector는 목각인형의 위치만 찾고, 이 분류기는 그 영역의 윤곽 방향 분포가
학습된 정상/낙상 경계 중 어느 쪽에 있는지 판단한다. 사람 Pose가 목각인형에서
불안정할 때 사용하는 전용 자세 판정기다.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


HOG_SIZE = (64, 64)
# 64x64 입력, 8x8 셀, 2x2 셀 블록, 셀당 9방향이므로 특징 수는
# (8-1) * (8-1) * 2 * 2 * 9 = 1,764개다.


def crop_with_padding(frame: np.ndarray, box, padding: float = 0.35):
    """팔·다리가 잘리지 않도록 bbox 사방에 비율 여백을 더해 자른다."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = (float(value) for value in box)
    pad_x = max(x2 - x1, 1.0) * padding
    pad_y = max(y2 - y1, 1.0) * padding
    left, top = max(0, int(x1 - pad_x)), max(0, int(y1 - pad_y))
    right, bottom = min(width, int(x2 + pad_x)), min(height, int(y2 + pad_y))
    return frame[top:bottom, left:right]


def hog_features(crop: np.ndarray) -> np.ndarray:
    """crop을 SVM 입력 형식인 ``(1, 1764) float32`` HOG 배열로 바꾼다."""
    if crop is None or crop.size == 0:
        raise ValueError("crop must not be empty")
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, HOG_SIZE, interpolation=cv2.INTER_AREA)
    # 히스토그램 평활화로 밝기 차이를 줄여 윤곽 방향에 더 집중하게 한다.
    gray = cv2.equalizeHist(gray)
    descriptor = cv2.HOGDescriptor(
        HOG_SIZE, (16, 16), (8, 8), (8, 8), 9
    )
    return descriptor.compute(gray).reshape(1, -1).astype(np.float32)


class PostureClassifier:
    """OpenCV SVM 파일을 로드해 bbox crop의 낙상 여부를 판정한다."""

    def __init__(self, model_path: str | Path):
        path = Path(model_path).expanduser().resolve()
        if not path.is_file():
            raise RuntimeError(f"posture classifier not found: {path}")
        self.model = cv2.ml.SVM_load(str(path))

    def predict_fallen(self, crop: np.ndarray) -> bool:
        """SVM 클래스 1은 낙상, 클래스 0은 비낙상으로 해석한다."""
        _, prediction = self.model.predict(hog_features(crop))
        return int(prediction[0, 0]) == 1
