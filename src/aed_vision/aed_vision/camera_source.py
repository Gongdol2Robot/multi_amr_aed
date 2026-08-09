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
    """카메라 설정값을 OpenCV가 열 수 있는 장치 번호 또는 경로로 변환한다.

    ``auto``이면 내장 카메라를 제외한 외장 USB 웹캠을 탐색하며, 후보가
    없거나 여러 대라서 하나를 확정할 수 없으면 명시적인 설정을 요구한다.
    """
    if value != "auto":
        return int(value) if value.isdigit() else value

    candidates = sorted(Path("/dev/v4l/by-id").glob("*-video-index0"))
    external = [
        path for path in candidates
        if not any(
            marker in path.name.lower()
            for marker in _INTERNAL_CAMERA_MARKERS
        )
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
        """카메라를 열고 원본 영상 publisher와 프레임 읽기 타이머를 만든다."""
        self.node = node
        self.on_frame = on_frame
        value = str(self._param(device_parameter))
        device = _resolve_camera_device(value)
        width = int(self._param("width"))
        height = int(self._param("height"))
        fps = float(self._param("fps"))

        self.capture = self._open_capture(device)
        self._configure_capture(width, height, fps)
        self.publisher = node.create_publisher(
            CompressedImage, image_topic, CAMERA_QOS
        )
        self.timer = node.create_timer(1.0 / max(fps, 1.0), self._read)
        node.get_logger().info(
            f"Direct webcam {device}: {width}x{height}@{fps:.1f}"
        )

    def _param(self, name: str):
        """부모 노드에 선언된 카메라 파라미터 값을 반환한다."""
        return self.node.get_parameter(name).value

    @staticmethod
    def _open_capture(device):
        """V4L2를 우선 사용하고 실패하면 OpenCV 기본 백엔드로 연다."""
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            capture = cv2.VideoCapture(device)
        if not capture.isOpened():
            raise RuntimeError(f"Unable to open webcam: device={device}")
        return capture

    def _configure_capture(self, width: int, height: int, fps: float) -> None:
        """USB 지연을 줄이는 형식·해상도·FPS·버퍼 설정을 적용한다."""
        self.capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")
        )
        for prop, setting in (
            (cv2.CAP_PROP_FRAME_WIDTH, width),
            (cv2.CAP_PROP_FRAME_HEIGHT, height),
            (cv2.CAP_PROP_FPS, fps),
            # 드라이버 내부 버퍼를 1프레임으로 제한해, 처리가 느려져도 오래된
            # 프레임이 아니라 항상 최신 프레임을 읽게 한다(지연 누적 방지).
            (cv2.CAP_PROP_BUFFERSIZE, 1),
        ):
            self.capture.set(prop, setting)

    def _read(self) -> None:
        """최신 카메라 프레임을 읽어 JPEG 토픽과 추론 콜백으로 전달한다."""
        success, frame = self.capture.read()
        if not success:
            self.node.get_logger().warning(
                "Failed to read direct webcam frame"
            )
            return
        # 모니터링용으로 원본 프레임을 JPEG 압축해 토픽으로도 발행한다
        # (다른 노트북/터미널에서 rqt_image_view 등으로 원격 확인 가능).
        quality = int(self._param("jpeg_quality"))
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        message = self._compressed_message(encoded if ok else None)
        if not ok:
            self.node.get_logger().warning(
                "Failed to encode direct webcam frame"
            )
        else:
            self.publisher.publish(message)
        # 디코딩된 원본 numpy 프레임을 콜백으로 바로 넘겨, 추론 쪽에서 다시
        # JPEG를 디코딩하지 않아도 되게 한다(같은 프로세스이므로 왕복 불필요).
        self.on_frame(frame, message)

    def _compressed_message(self, encoded=None) -> CompressedImage:
        """인코딩된 JPEG와 현재 시각으로 ROS 메시지를 만든다."""
        message = CompressedImage()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = str(self._param("frame_id"))
        message.format = "jpeg"
        if encoded is not None:
            message.data = encoded.tobytes()
        return message

    def close(self) -> None:
        """노드 종료 시 OpenCV 카메라 장치를 해제한다."""
        self.capture.release()
