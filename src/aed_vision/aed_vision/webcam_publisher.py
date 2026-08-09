"""YOLO 추론 없이 USB 웹캠 프레임을 JPEG 압축 토픽으로 발행한다."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from .camera_source import DirectCameraSource


class WebcamPublisher(Node):
    """로컬 USB 웹캠을 읽어 압축 영상 스트림만 발행하는 ROS 2 노드."""

    def __init__(self) -> None:
        super().__init__("webcam_publisher")
        self.declare_parameter("device", "auto")
        self.declare_parameter(
            "image_topic", "/camera_alley/image_raw/compressed"
        )
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "camera_alley_optical_frame")
        self.declare_parameter("jpeg_quality", 85)

        image_topic = str(self.get_parameter("image_topic").value)
        self.camera_source = DirectCameraSource(
            self,
            image_topic,
            self._ignore_frame,
            device_parameter="device",
        )
        self.get_logger().info(
            f"Publishing compressed webcam images only: {image_topic}"
        )

    @staticmethod
    def _ignore_frame(_frame, _message) -> None:
        """DirectCameraSource가 발행을 마친 로컬 프레임은 추가 처리하지 않는다."""

    def destroy_node(self) -> bool:
        self.camera_source.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
