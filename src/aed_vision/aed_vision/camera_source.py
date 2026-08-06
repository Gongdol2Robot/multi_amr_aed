"""OpenCV USB 카메라 입력과 ROS 압축 원본 영상 발행."""

from __future__ import annotations

import cv2
from sensor_msgs.msg import CompressedImage

from .qos import CAMERA_QOS


class DirectCameraSource:
    def __init__(self, node, image_topic: str, on_frame) -> None:
        self.node = node
        self.on_frame = on_frame
        value = str(node.get_parameter("camera_device").value)
        device = int(value) if value.isdigit() else value
        width = int(node.get_parameter("width").value)
        height = int(node.get_parameter("height").value)
        fps = float(node.get_parameter("fps").value)

        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open webcam: device={device}")

        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        for prop, setting in (
            (cv2.CAP_PROP_FRAME_WIDTH, width),
            (cv2.CAP_PROP_FRAME_HEIGHT, height),
            (cv2.CAP_PROP_FPS, fps),
            (cv2.CAP_PROP_BUFFERSIZE, 1),
        ):
            self.capture.set(prop, setting)

        self.publisher = node.create_publisher(
            CompressedImage, image_topic, CAMERA_QOS
        )
        self.timer = node.create_timer(1.0 / max(fps, 1.0), self._read)
        node.get_logger().info(
            f"Direct webcam {device}: {width}x{height}@{fps:.1f}"
        )

    def _read(self) -> None:
        success, frame = self.capture.read()
        if not success:
            self.node.get_logger().warning("Failed to read direct webcam frame")
            return
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
        self.on_frame(frame, message)

    def close(self) -> None:
        self.capture.release()
