from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from backend.ros.images import raw_image_to_jpeg
from backend.ros.topics import DEFAULT_STREAMS


def test_camera1_stream_uses_network_compressed_vision() -> None:
    streams = {source.stream_id: source for source in DEFAULT_STREAMS}

    assert streams["camera_open"].topic == (
        "/camera_open/vision/debug/compressed"
    )
    assert streams["camera_open"].compressed is True


def test_robot_streams_use_compressed_vision_preview() -> None:
    robots = {source.stream_id: source for source in DEFAULT_STREAMS
              if source.kind == "robot"}
    assert robots["robot1"].topic == "/robot1/vision/debug/compressed"
    assert robots["robot2"].topic == "/robot2/vision/debug/compressed"
    assert robots["robot1"].compressed is True
    assert robots["robot2"].compressed is True


def test_rgb_preview_with_row_padding_becomes_jpeg() -> None:
    # Two RGB pixels plus two padding bytes per row.
    message = SimpleNamespace(
        encoding="rgb8",
        width=2,
        height=1,
        step=8,
        data=bytes([255, 0, 0, 0, 255, 0, 99, 99]),
    )
    jpeg = raw_image_to_jpeg(message, quality=90)
    decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (1, 2, 3)


def test_unknown_preview_encoding_is_rejected() -> None:
    message = SimpleNamespace(
        encoding="16UC1", width=1, height=1, step=2, data=b"\x00\x00"
    )
    with pytest.raises(ValueError, match="encoding"):
        raw_image_to_jpeg(message)
