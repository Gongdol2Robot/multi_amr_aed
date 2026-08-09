"""sensor_msgs/Image를 HMI가 중계할 JPEG로 변환한다."""

import cv2
import numpy as np


_ENCODINGS = {
    "bgr8": (3, None),
    "rgb8": (3, cv2.COLOR_RGB2BGR),
    "bgra8": (4, cv2.COLOR_BGRA2BGR),
    "rgba8": (4, cv2.COLOR_RGBA2BGR),
    "mono8": (1, None),
}


def raw_image_to_jpeg(message, quality: int = 70) -> bytes:
    """Convert a common 8-bit ROS image encoding to a compact JPEG.

    ``step`` may include row padding, so reshape by step first and discard the
    padding before interpreting pixels.
    """
    encoding = str(message.encoding).lower()
    if encoding not in _ENCODINGS:
        raise ValueError(f"지원하지 않는 Image encoding: {message.encoding}")

    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    channels, color_conversion = _ENCODINGS[encoding]
    row_bytes = width * channels
    if height <= 0 or width <= 0 or step < row_bytes:
        raise ValueError(
            f"잘못된 Image 크기: {width}x{height}, step={step}"
        )

    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected = height * step
    if raw.size < expected:
        raise ValueError(
            f"Image 데이터 부족: expected={expected}, actual={raw.size}"
        )

    rows = raw[:expected].reshape(height, step)[:, :row_bytes]
    if channels == 1:
        image = rows.reshape(height, width)
    else:
        image = rows.reshape(height, width, channels)
    if color_conversion is not None:
        image = cv2.cvtColor(image, color_conversion)

    ok, encoded = cv2.imencode(
        ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
    )
    if not ok:
        raise ValueError("JPEG 인코딩 실패")
    return encoded.tobytes()
