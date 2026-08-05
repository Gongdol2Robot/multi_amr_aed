"""Publish a local webcam as a JPEG-compressed ROS image topic."""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage


CAMERA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class WebcamPublisher(Node):
    """Read a webcam and publish JPEG frames."""

    def __init__(self) -> None:
        super().__init__("webcam_publisher")
        self.declare_parameter("device", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter(
            "image_topic", "/aed/camera/image_raw/compressed"
        )
        self.declare_parameter("frame_id", "aed_camera_optical_frame")
        self.declare_parameter("jpeg_quality", 80)

        device = int(self.get_parameter("device").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")

        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open webcam: device={device}")

        self.capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG"),
        )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.publisher = self.create_publisher(
            CompressedImage, topic, CAMERA_QOS
        )
        self.read_count = 0
        self.read_failures = 0
        self.publish_count = 0
        self.timer = self.create_timer(1.0 / max(fps, 1.0), self._publish)
        self.stats_timer = self.create_timer(5.0, self._report_stats)
        self.get_logger().info(
            f"webcam {device}: {width}x{height}@{fps:.1f} -> {topic} "
            f"(JPEG quality={self.jpeg_quality})"
        )

    def _publish(self) -> None:
        success, frame = self.capture.read()
        if not success:
            self.read_failures += 1
            return
        self.read_count += 1
        encoded_ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not encoded_ok:
            self.get_logger().warning("Failed to encode JPEG frame")
            return

        message = CompressedImage()
        message.format = "jpeg"
        message.data = encoded.tobytes()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)
        self.publish_count += 1

    def _report_stats(self) -> None:
        self.get_logger().info(
            f"camera stats: read={self.read_count}, "
            f"read_fail={self.read_failures}, "
            f"publish={self.publish_count} / 5 sec"
        )
        self.read_count = 0
        self.read_failures = 0
        self.publish_count = 0

    def destroy_node(self) -> None:
        self.capture.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

