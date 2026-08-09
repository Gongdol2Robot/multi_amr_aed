"""경량 crop 자세 분류기의 전처리 테스트."""

import numpy as np
import pytest

from aed_vision.posture_classifier import crop_with_padding, hog_features


def test_hog_features_have_fixed_shape() -> None:
    features = hog_features(np.zeros((80, 120, 3), dtype=np.uint8))
    assert features.shape == (1, 1764)
    assert features.dtype == np.float32


def test_crop_padding_stays_inside_frame() -> None:
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    crop = crop_with_padding(frame, (0, 0, 30, 40), padding=0.5)
    assert 0 < crop.shape[0] <= 100
    assert 0 < crop.shape[1] <= 100


def test_hog_rejects_empty_crop() -> None:
    with pytest.raises(ValueError, match="empty"):
        hog_features(np.empty((0, 0, 3), dtype=np.uint8))
