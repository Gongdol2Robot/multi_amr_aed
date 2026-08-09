#!/usr/bin/env python3
"""mp4 를 CompressedImage 로 실제 ROS 토픽에 흘린다.

왜 bag 대신 이것인가
--------------------
녹화해 둔 bag 을 그대로 틀면 두 가지가 아쉽다.

* 로봇 주행 bag 의 OAK-D 영상에는 **검출 상자가 없다.** 그때는 로봇 쪽
  검출 노드가 없었다. 관제 화면에 벽만 흐른다.
* 웹캠 bag 은 11분짜리라 앞쪽 대부분이 인형이 **서 있는** 구간이다.
  재생을 걸어 두면 정작 쓰러진 장면이 잘 안 나온다.

그래서 그 bag 들에서 쓸 만한 구간만 골라 만들어 둔 mp4(docs/videos)를
쓴다. 화면에 보이는 그림은 결국 같은 카메라로 찍은 같은 검출 결과이고,
좋은 구간만 남은 것뿐이다.

**중요한 것은 여기가 여전히 실제 ROS 경로라는 점이다.** 프레임이 파일에서
오든 bag 에서 오든, 관제는 `sensor_msgs/CompressedImage` 를 실제 토픽에서
구독한다. 메시지 타입·QoS·구독이 붙는지가 모두 실물과 같이 검증된다.

검출 수도 함께 낸다. `tools/scan_detections.py` 가 남긴 사이드카를 읽어
재생 중인 프레임의 검출 수를 `person_count` 로 흘린다. 화면의 상자와
관제의 숫자가 어긋나지 않는다.

사용:
  python3 tools/video_publisher.py \\
      --stream camera_open=docs/videos/camera_open_demo.mp4 \\
      --stream robot1=docs/videos/robot_approach_yolo.mp4
"""
import argparse
import bisect
import json
import os
import sys
import time

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
from std_msgs.msg import UInt32

# 관제(backend/ros/topics.py)의 image_qos, vision_detector 의 CAMERA_QOS 와
# 같아야 한다. 다르면 구독이 아예 안 붙는다.
IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
COUNT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

ROBOT_STREAMS = ("robot1", "robot2")


def image_topic(stream_id: str) -> str:
    """관제가 그 갈래를 어느 토픽에서 찾는지.

    로봇은 OAK-D 원본 자리에, 고정 웹캠은 vision_detector 의 debug 영상
    자리에 낸다. backend/ros/topics.py 의 DEFAULT_STREAMS 와 같아야 한다.
    """
    if stream_id in ROBOT_STREAMS:
        return f"/{stream_id}/oakd/rgb/image_raw/compressed"
    return f"/{stream_id}/vision/debug/compressed"


def count_topic(stream_id: str) -> str:
    return f"/{stream_id}/vision/person_count"


class _Track:
    """영상 시각 -> 그 시점의 검출 수. 사이드카가 없으면 늘 0."""

    def __init__(self, path: str) -> None:
        self.times: list[float] = []
        self.counts: list[int] = []
        if not os.path.exists(path):
            return
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
        for time_s, fallen, helper in data.get("samples", []):
            self.times.append(float(time_s))
            self.counts.append(int(fallen) + int(helper))

    def at(self, seconds: float) -> int:
        if not self.times:
            return 0
        # 물어본 시각 이하의 마지막 표본. 표본 사이에서는 직전 값을 쓴다.
        index = bisect.bisect_right(self.times, seconds) - 1
        return self.counts[max(index, 0)]


class _Stream:
    def __init__(self, node: Node, stream_id: str, path: str,
                 start_ratio: float) -> None:
        self.stream_id = stream_id
        self.capture = cv2.VideoCapture(path)
        if not self.capture.isOpened():
            raise SystemExit(f"실패: {path} 를 못 엶")
        self.fps = self.capture.get(cv2.CAP_PROP_FPS) or 15.0
        total = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
        self.index = 0
        if total > 0 and start_ratio > 0.0:
            # 두 로봇이 같은 파일을 쓸 때 같은 그림이 나란히 뜨지 않게 한다.
            self.index = int(total * start_ratio)
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, self.index)

        self.track = _Track(os.path.splitext(path)[0] + ".detections.json")
        self.image_pub = node.create_publisher(
            CompressedImage, image_topic(stream_id), IMAGE_QOS
        )
        self.count_pub = node.create_publisher(
            UInt32, count_topic(stream_id), COUNT_QOS
        )
        node.get_logger().info(
            f"{stream_id}: {os.path.basename(path)} {self.fps:.0f}fps "
            f"-> {image_topic(stream_id)}"
        )

    def tick(self, node: Node) -> None:
        position = self.index / self.fps
        got, frame = self.capture.read()
        if got:
            self.index += 1
        else:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.index = 0
            position = 0.0
            got, frame = self.capture.read()
            if not got:
                return
            self.index = 1

        ok, buffer = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
        )
        if not ok:
            return

        stamp = node.get_clock().now().to_msg()
        message = CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = self.stream_id
        message.format = "jpeg"
        message.data = buffer.tobytes()
        self.image_pub.publish(message)

        count = UInt32()
        count.data = int(self.track.at(position))
        self.count_pub.publish(count)

    def release(self) -> None:
        self.capture.release()


class VideoPublisher(Node):
    def __init__(self, specs: list[tuple[str, str, float]], fps: float) -> None:
        super().__init__("aed_video_publisher")
        self.streams = [
            _Stream(self, stream_id, path, start_ratio)
            for stream_id, path, start_ratio in specs
        ]
        self.create_timer(1.0 / fps, self._tick)

    def _tick(self) -> None:
        for stream in self.streams:
            stream.tick(self)

    def release(self) -> None:
        for stream in self.streams:
            stream.release()


def main() -> int:
    parser = argparse.ArgumentParser(description="mp4 를 ROS 토픽으로 낸다")
    parser.add_argument(
        "--stream", action="append", default=[], metavar="ID=파일[:비율]",
        help="갈래와 영상. 비율은 재생 시작 지점(0~1). "
             "예: robot2=docs/videos/robot_approach_yolo.mp4:0.45",
    )
    parser.add_argument("--fps", type=float, default=15.0)
    args = parser.parse_args()

    if not args.stream:
        print("실패: --stream 이 하나도 없습니다")
        return 1

    specs = []
    for item in args.stream:
        if "=" not in item:
            print(f"실패: --stream {item} 의 형식이 ID=파일 이 아닙니다")
            return 1
        stream_id, _, rest = item.partition("=")
        path, _, ratio = rest.partition(":")
        if not os.path.exists(path):
            print(f"실패: {path} 가 없습니다")
            return 1
        specs.append((stream_id, path, float(ratio) if ratio else 0.0))

    rclpy.init()
    node = VideoPublisher(specs, args.fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
