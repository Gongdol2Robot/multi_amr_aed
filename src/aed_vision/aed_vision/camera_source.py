"""OpenCV USB 카메라 입력과 ROS 압축 원본 영상 발행."""

from __future__ import annotations

from pathlib import Path

import cv2
from sensor_msgs.msg import CompressedImage

from .qos import CAMERA_QOS


_INTERNAL_CAMERA_MARKERS = (
    "integrated",
    "built-in",
    "builtin",
    "hp_wide_vision",
    "facetime",
    "front_camera",
    "ir_camera",
)


def _resolve_camera_device(value: str) -> str | int:
    """Resolve ``auto`` to one unambiguous external USB webcam."""
    if value != "auto":
        return int(value) if value.isdigit() else value

    candidates = sorted(Path("/dev/v4l/by-id").glob("*-video-index0"))
    external = [
        path for path in candidates
        if not any(marker in path.name.lower() for marker in _INTERNAL_CAMERA_MARKERS)
    ]
    if len(external) == 1:
        return str(external[0])
    if not external:
        raise RuntimeError(
            "No external USB webcam found under /dev/v4l/by-id; "
            "set camera_device explicitly"
        )
    names = ", ".join(str(path) for path in external)
    raise RuntimeError(
        f"Multiple external USB webcams found ({names}); "
        "set camera_device explicitly"
    )


class DirectCameraSource:
    def __init__(
        self,
        node,
        image_topic: str,
        on_frame,
        device_parameter: str = "camera_device",
    ) -> None:
        self.node = node
        self.on_frame = on_frame
        value = str(node.get_parameter(device_parameter).value)
        device = _resolve_camera_device(value)
        width = int(node.get_parameter("width").value)
        height = int(node.get_parameter("height").value)
        fps = float(node.get_parameter("fps").value)

        # V4L2 백엔드를 먼저 시도하고(리눅스 USB 웹캠에서 더 안정적), 실패하면
        # OpenCV 기본 백엔드로 재시도한다. 둘 다 실패하면 노드를 띄울 수 없으므로 예외.
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open webcam: device={device}")

        # MJPG로 캡처해야 USB 대역폭 안에서 해상도·FPS를 확보할 수 있다(YUYV는 무압축).
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        for prop, setting in (
            (cv2.CAP_PROP_FRAME_WIDTH, width),
            (cv2.CAP_PROP_FRAME_HEIGHT, height),
            (cv2.CAP_PROP_FPS, fps),
            # 드라이버 내부 버퍼를 1프레임으로 제한해, 처리가 느려져도 오래된
            # 프레임이 아니라 항상 최신 프레임을 읽게 한다(지연 누적 방지).
            (cv2.CAP_PROP_BUFFERSIZE, 1),
        ):
            self.capture.set(prop, setting)

        self.publisher = node.create_publisher(
            CompressedImage, image_topic, CAMERA_QOS
        )
        # 카메라 fps 주기로 타이머를 돌려 폴링 방식으로 프레임을 읽는다.
        self.timer = node.create_timer(1.0 / max(fps, 1.0), self._read)
        node.get_logger().info(
            f"Direct webcam {device}: {width}x{height}@{fps:.1f}"
        )

    def _read(self) -> None:
        success, frame = self.capture.read()
        if not success:
            self.node.get_logger().warning("Failed to read direct webcam frame")
            return
        # 모니터링용으로 원본 프레임을 JPEG 압축해 토픽으로도 발행한다
        # (다른 노트북/터미널에서 rqt_image_view 등으로 원격 확인 가능).
        quality = int(self.node.get_parameter("jpeg_quality").value)
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if not ok:
            self.node.get_logger().warning("Failed to encode direct webcam frame")
            return

        message = CompressedImage()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = str(
            self.node.get_parameter("frame_id").value
        )
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.publisher.publish(message)
        # 디코딩된 원본 numpy 프레임을 콜백으로 바로 넘겨, 추론 쪽에서 다시
        # JPEG를 디코딩하지 않아도 되게 한다(같은 프로세스이므로 왕복 불필요).
        self.on_frame(frame, message)

    def close(self) -> None:
        self.capture.release()
