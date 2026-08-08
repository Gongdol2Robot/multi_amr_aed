"""Publish JPEG-compressed frames from a webcam without running inference."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from .camera_source import DirectCameraSource


class WebcamPublisher(Node):
    """Read a local webcam and publish only its compressed image stream."""

    def __init__(self) -> None:
        super().__init__("webcam_publisher")
        self.declare_parameter("device", "/dev/video2")
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
        """Discard the local frame after DirectCameraSource publishes it."""

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
