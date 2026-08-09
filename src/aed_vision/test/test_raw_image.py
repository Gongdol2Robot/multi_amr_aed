"""sensor_msgs/Image의 BGR 변환 테스트."""

from types import SimpleNamespace

import numpy as np
import pytest

from aed_vision.vision_detector import raw_image_to_bgr


def _message(encoding, width, height, step, data):
    return SimpleNamespace(
        encoding=encoding,
        width=width,
        height=height,
        step=step,
        data=data,
    )


def test_rgb_image_is_converted_to_bgr_with_row_padding() -> None:
    message = _message("rgb8", 1, 1, 4, bytes([10, 20, 30, 255]))

    frame = raw_image_to_bgr(message)

    np.testing.assert_array_equal(frame, [[[30, 20, 10]]])


def test_mono_image_is_expanded_to_three_channels() -> None:
    frame = raw_image_to_bgr(_message("mono8", 1, 1, 1, bytes([42])))

    np.testing.assert_array_equal(frame, [[[42, 42, 42]]])


def test_raw_image_rejects_bad_encoding_step_and_buffer() -> None:
    with pytest.raises(ValueError, match="unsupported"):
        raw_image_to_bgr(_message("16UC1", 1, 1, 2, bytes(2)))
    with pytest.raises(ValueError, match="step"):
        raw_image_to_bgr(_message("bgr8", 2, 1, 5, bytes(5)))
    with pytest.raises(ValueError, match="short"):
        raw_image_to_bgr(_message("bgr8", 1, 2, 3, bytes(3)))
