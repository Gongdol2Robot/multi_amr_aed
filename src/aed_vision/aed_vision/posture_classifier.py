"""mannequin detector crop을 FALLEN/NON_FALLEN으로 분류하는 경량 모델."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


HOG_SIZE = (64, 64)


def crop_with_padding(frame: np.ndarray, box, padding: float = 0.35):
    """원본 프레임에서 bbox에 비율 여백을 추가해 crop한다."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = (float(value) for value in box)
    pad_x = max(x2 - x1, 1.0) * padding
    pad_y = max(y2 - y1, 1.0) * padding
    left, top = max(0, int(x1 - pad_x)), max(0, int(y1 - pad_y))
    right, bottom = min(width, int(x2 + pad_x)), min(height, int(y2 + pad_y))
    return frame[top:bottom, left:right]


def hog_features(crop: np.ndarray) -> np.ndarray:
    """조명 영향을 줄인 64x64 HOG 특징을 한 행의 float32로 반환한다."""
    if crop is None or crop.size == 0:
        raise ValueError("crop must not be empty")
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, HOG_SIZE, interpolation=cv2.INTER_AREA)
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
        _, prediction = self.model.predict(hog_features(crop))
        return int(prediction[0, 0]) == 1
