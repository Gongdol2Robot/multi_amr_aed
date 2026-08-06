"""로컬 USB 웹캠 프레임을 JPEG 압축 ROS 2 영상 토픽으로 발행한다."""

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
    # 실시간 영상은 모든 과거 프레임보다 가장 최신 프레임 하나가 중요하다.
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    # 일부 프레임 손실을 허용해 재전송으로 인한 지연 누적을 피한다.
    reliability=ReliabilityPolicy.BEST_EFFORT,
    # 늦게 접속한 구독자에게 오래된 영상 프레임을 다시 보내지 않는다.
    durability=DurabilityPolicy.VOLATILE,
)


class WebcamPublisher(Node):
    """USB 웹캠을 열고 설정된 주기로 JPEG CompressedImage를 발행한다."""

    def __init__(self) -> None:
        """카메라 파라미터를 읽고 VideoCapture, publisher, timer를 준비한다."""
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

        # Linux에서는 V4L2를 우선 지정해 backend 선택의 변동을 줄인다.
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            # 장치나 OpenCV 빌드가 V4L2 지정을 지원하지 않으면 자동 backend로
            # 한 번 더 시도한다.
            self.capture.release()
            self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open webcam: device={device}")

        # USB 대역폭을 줄이기 위해 카메라가 지원하면 MJPEG 출력을 요청한다.
        self.capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG"),
        )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        # 드라이버가 지원하는 경우 버퍼를 1로 줄여 밀린 과거 영상 대신 최신
        # 프레임을 읽도록 한다. set() 실패는 장치별 차이이므로 치명 오류가 아니다.
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
        """타이머마다 카메라 한 프레임을 읽어 JPEG 메시지로 발행한다.

        읽기나 인코딩이 실패한 프레임은 버리고 다음 타이머 주기에서 다시
        시도한다. header stamp는 카메라를 읽은 현재 ROS 시간을 사용한다.
        """
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
        """5초 동안의 읽기·실패·발행 횟수를 로그로 남기고 카운터를 초기화한다."""
        self.get_logger().info(
            f"camera stats: read={self.read_count}, "
            f"read_fail={self.read_failures}, "
            f"publish={self.publish_count} / 5 sec"
        )
        self.read_count = 0
        self.read_failures = 0
        self.publish_count = 0

    def destroy_node(self) -> None:
        """노드 종료 전에 USB 카메라 장치를 명시적으로 해제한다."""
        self.capture.release()
        super().destroy_node()


def main(args=None) -> None:
    """ROS2 context와 웹캠 노드를 실행하고 종료 시 카메라를 정리한다."""
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
