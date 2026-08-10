#!/usr/bin/env python3
"""TurtleBot OAK-D RGB 토픽으로 학습된 YOLO 모델을 실시간 테스트한다.

기본 입력은 ``/robot2/oakd/rgb/preview/image_raw``이다. 화면에서 q 또는
ESC를 누르면 종료한다.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path


TRAINING_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = (
    TRAINING_ROOT
    / "finetune_runs"
    / "yolo11n_20260805_231945_hard_negative_20260806_115828"
    / "weights"
    / "best.pt"
)
DEFAULT_TOPIC = "/robot2/oakd/rgb/preview/image_raw"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--weights",
        type=Path,
        default=DEFAULT_WEIGHTS,
        help=f"YOLO .pt 또는 .engine 가중치 (기본: {DEFAULT_WEIGHTS})",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"OAK-D preview Image 토픽 (기본: {DEFAULT_TOPIC})",
    )
    parser.add_argument("--conf", type=float, default=0.4)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument(
        "--device", default=None, help="추론 장치: 0, 1, cpu 등 (기본: 자동)"
    )
    parser.add_argument(
        "--target-class",
        default=None,
        help="표시할 클래스 하나만 지정 (예: fallen_person, 기본: 모든 클래스)",
    )
    parser.add_argument(
        "--max-det", type=int, default=100, help="프레임당 최대 검출 수"
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> Path:
    if not 0.0 <= args.conf <= 1.0:
        raise SystemExit("--conf는 0 이상 1 이하여야 합니다.")
    if not 0.0 <= args.iou <= 1.0:
        raise SystemExit("--iou는 0 이상 1 이하여야 합니다.")
    if args.imgsz < 32:
        raise SystemExit("--imgsz는 32 이상이어야 합니다.")
    if args.max_det < 1:
        raise SystemExit("--max-det는 1 이상이어야 합니다.")
    weights = args.weights.expanduser().resolve()
    if not weights.is_file():
        raise SystemExit(
            f"가중치를 찾지 못했습니다: {weights}\n"
            "다른 모델은 --weights /경로/best.pt 로 지정하세요."
        )
    return weights


def main() -> int:
    args = parse_args()
    weights = validate_args(args)

    try:
        import cv2
        import rclpy
        from aed_vision.vision_detector import raw_image_to_bgr
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import Image
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            f"필요한 패키지를 불러오지 못했습니다: {exc}\n"
            "ROS 2 환경을 source하고 "
            "vision_training/requirements.txt를 설치하세요."
        ) from exc

    class TurtleBotYoloTest(Node):
        def __init__(self) -> None:
            super().__init__("turtlebot_yolo_test")
            self.model = YOLO(str(weights), task="detect")
            self.options = {
                "conf": args.conf,
                "iou": args.iou,
                "imgsz": args.imgsz,
                "max_det": args.max_det,
                "verbose": False,
            }
            if args.device is not None:
                self.options["device"] = args.device
            if args.target_class:
                class_ids = [
                    class_id
                    for class_id, name in self.model.names.items()
                    if name == args.target_class
                ]
                if not class_ids:
                    available = ", ".join(self.model.names.values())
                    raise ValueError(
                        f"클래스 '{args.target_class}'가 없습니다. 사용 가능: {available}"
                    )
                self.options["classes"] = class_ids

            self.window_name = "TurtleBot OAK-D | YOLO Test"
            self.last_time = None
            self.fps = 0.0
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.subscription = self.create_subscription(
                Image, args.topic, self.image_callback, qos
            )
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            self.get_logger().info(f"모델: {weights}")
            self.get_logger().info(f"입력: {args.topic}")
            self.get_logger().info("종료: 화면에서 q 또는 ESC")

        def image_callback(self, message: Image) -> None:
            try:
                frame = raw_image_to_bgr(message)
            except ValueError as error:
                self.get_logger().warning(
                    f"preview 이미지를 변환하지 못했습니다: {error}",
                    throttle_duration_sec=5.0,
                )
                return

            result = self.model.predict(frame, **self.options)[0]
            canvas = result.plot()
            now = time.perf_counter()
            if self.last_time is not None:
                instant_fps = 1.0 / max(now - self.last_time, 1e-6)
                self.fps = (
                    instant_fps
                    if self.fps == 0.0
                    else 0.9 * self.fps + 0.1 * instant_fps
                )
            self.last_time = now
            count = 0 if result.boxes is None else len(result.boxes)
            text = f"detections: {count} | FPS: {self.fps:.1f}"
            cv2.putText(
                canvas, text, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, (0, 255, 255), 2, cv2.LINE_AA,
            )
            cv2.imshow(self.window_name, canvas)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                rclpy.shutdown()

        def destroy_node(self) -> None:
            cv2.destroyAllWindows()
            super().destroy_node()

    rclpy.init()
    node = None
    try:
        node = TurtleBotYoloTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
