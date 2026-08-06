"""고정 카메라 영상에서 구조 대상과 선택적으로 골목 혼잡도를 검출한다."""

from __future__ import annotations

import json
import os
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from aed_interfaces.msg import EmergencyEvent, Heartbeat
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, UInt32

from .camera_source import DirectCameraSource
from .detection_logic import (
    Box,
    TemporalConfirmation,
)
from .inference_pipeline import InferenceOutput, InferencePipeline
from .qos import CAMERA_QOS


class VisionDetector(Node):
    """압축 영상을 받아 카메라 설치 장소에 맞는 추론 파이프라인을 수행한다.

    open 모드는 구조 모델 하나만 실행하고, alley 모드는 같은 프레임에 구조
    모델과 COCO person 모델을 실행한다. 두 역할을 하나의 클래스로 구현해
    코드 중복을 막고 YAML의 ``mode``만으로 배치 장소를 구분한다.
    """

    def __init__(self) -> None:
        """파라미터, 모델, ROS publisher/subscriber와 타이머를 초기화한다."""
        super().__init__("vision_detector")
        self._declare_parameters()

        # 카메라 식별자는 토픽 경로와 이벤트 출처에 함께 사용한다.
        self.camera_id = str(self.get_parameter("camera_id").value)
        self.zone_id = str(self.get_parameter("zone_id").value)
        self.mode = str(self.get_parameter("mode").value).lower()
        if self.mode not in ("open", "alley"):
            raise ValueError("mode must be 'open' or 'alley'")
        # 인파 모델은 연산량을 줄이기 위해 alley 모드에서만 메모리에 올린다.
        self.enable_crowd = self.mode == "alley"
        self.frame_id = str(self.get_parameter("location_frame_id").value)
        self.location_x = float(self.get_parameter("location_x").value)
        self.location_y = float(self.get_parameter("location_y").value)
        self.crowd_roi = list(self.get_parameter("crowd_roi").value)
        self.crowded_threshold = int(
            self.get_parameter("crowded_person_threshold").value
        )
        self.overlap_threshold = float(
            self.get_parameter("fallen_person_overlap_iou").value
        )
        window = int(self.get_parameter("confirmation_window").value)
        hits = int(self.get_parameter("confirmation_hits").value)
        self.confirmation = TemporalConfirmation(window, hits)
        # was_confirmed는 매 프레임 이벤트를 반복 발행하지 않고 상태가 바뀔 때만
        # CONFIRMED/CANCELED 이벤트를 한 번씩 발행하기 위한 이전 상태이다.
        self.was_confirmed = False
        self.event_id = ""
        self.sequence = 0
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = f"AED Vision - {self.camera_id} ({self.mode})"
        self.processed_frames = 0
        self.first_frame_logged = False
        # 추론보다 카메라 발행 주기가 빠를 때 콜백이 중첩되지 않게 하는 보호값.
        self.busy = False

        # 첫 YOLO 추론은 CUDA 초기화 때문에 수 초 걸릴 수 있다. 첫 결과를 기다린
        # 뒤 창을 만들면 실행 여부를 알기 어려우므로 노드 시작 즉시 창을 만든다.
        if self.show_window:
            if not os.environ.get("DISPLAY") and not os.environ.get(
                "WAYLAND_DISPLAY"
            ):
                self.get_logger().warning(
                    "show_window=true but DISPLAY/WAYLAND_DISPLAY is not set; "
                    "disabling the local window"
                )
                self.show_window = False
            else:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 960, 720)
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    placeholder,
                    "Loading YOLO model / waiting for camera...",
                    (45, 245),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(self.window_name, placeholder)
                cv2.waitKey(1)

        self.pipeline = InferencePipeline(
            rescue_weights=str(self.get_parameter("rescue_weights").value),
            person_weights=str(self.get_parameter("person_weights").value),
            enable_crowd=self.enable_crowd,
            rescue_conf=float(self.get_parameter("rescue_conf").value),
            person_conf=float(self.get_parameter("person_conf").value),
            iou=float(self.get_parameter("iou").value),
            imgsz=int(self.get_parameter("imgsz").value),
            device=str(self.get_parameter("inference_device").value),
            crowd_roi=self.crowd_roi,
            crowded_threshold=self.crowded_threshold,
            overlap_threshold=self.overlap_threshold,
        )

        # 절대 토픽명을 사용해 두 노트북이 같은 ROS_DOMAIN_ID에 있어도
        # camera_open과 camera_alley 결과가 명확하게 분리되도록 한다.
        prefix = f"/{self.camera_id}/vision"
        self.event_pub = self.create_publisher(
            EmergencyEvent, f"{prefix}/emergency_event", 10
        )
        self.status_pub = self.create_publisher(String, f"{prefix}/status", 10)
        self.crowd_pub = self.create_publisher(
            String, f"{prefix}/crowd_level", 10
        )
        self.person_count_pub = self.create_publisher(
            UInt32, f"{prefix}/person_count", 10
        )
        self.heartbeat_pub = self.create_publisher(
            Heartbeat, f"{prefix}/heartbeat", 10
        )
        self.debug_pub = self.create_publisher(
            CompressedImage, f"{prefix}/debug/compressed", CAMERA_QOS
        )
        image_topic = str(self.get_parameter("image_topic").value)
        self.subscription = self.create_subscription(
            CompressedImage, image_topic, self._on_image, CAMERA_QOS
        )
        # 기본 실행은 한 노드가 웹캠을 직접 읽는다. 같은 노트북 안에서 영상을
        # DDS로 왕복시키지 않아 화면 지연과 publisher/subscriber 연결 문제를 없앤다.
        self.direct_camera = bool(
            self.get_parameter("direct_camera").value
        )
        self.camera_source = None
        if self.direct_camera:
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.camera_source = DirectCameraSource(
                self, image_topic, self._process_frame
            )
        # 영상 유무와 관계없이 중앙 시스템이 노드 생존을 판단할 수 있게 한다.
        self.heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)
        self.get_logger().info(
            f"camera={self.camera_id} mode={self.mode} image={image_topic} "
            f"crowd_detection={self.enable_crowd}"
        )

    def _declare_parameters(self) -> None:
        """YAML로 덮어쓸 수 있는 모든 ROS 파라미터와 기본값을 선언한다.

        카메라별 차이는 코드 수정 없이 open_camera.yaml과 alley_camera.yaml로
        관리한다. 기본값만으로는 가중치가 없으므로 실제 실행 시 YAML이 필요하다.
        """
        self.declare_parameter("camera_id", "camera_open")
        self.declare_parameter("zone_id", "open_zone")
        self.declare_parameter("mode", "open")
        self.declare_parameter("image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("direct_camera", True)
        # camera_device는 USB 장치 경로, inference_device는 YOLO 연산 장치이다.
        # 내장 카메라가 /dev/video0을 차지하므로 기본 USB 웹캠은 video2로 둔다.
        self.declare_parameter("camera_device", "/dev/video2")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "aed_camera_optical_frame")
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("rescue_weights", "")
        self.declare_parameter("person_weights", "")
        self.declare_parameter("rescue_conf", 0.25)
        self.declare_parameter("person_conf", 0.25)
        self.declare_parameter("iou", 0.5)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("inference_device", "")
        self.declare_parameter("confirmation_window", 10)
        self.declare_parameter("confirmation_hits", 6)
        self.declare_parameter("crowd_roi", [0.0, 0.0, 1.0, 1.0])
        self.declare_parameter("crowded_person_threshold", 3)
        self.declare_parameter("fallen_person_overlap_iou", 0.4)
        self.declare_parameter("location_frame_id", "map")
        self.declare_parameter("location_x", 0.0)
        self.declare_parameter("location_y", 0.0)
        self.declare_parameter("publish_debug_image", True)
        # true이면 추론을 실행하는 노트북에 OpenCV 결과 창을 직접 표시한다.
        self.declare_parameter("show_window", True)
        self.declare_parameter("debug_jpeg_quality", 80)

    def _on_image(self, message: CompressedImage) -> None:
        """구독한 JPEG 프레임을 디코딩해 추론한다."""
        frame = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            self.get_logger().warning("Failed to decode compressed image")
            return
        self._process_frame(frame, message)

    def _process_frame(self, frame: np.ndarray, source: CompressedImage) -> None:
        """디코딩된 한 프레임의 구조·혼잡 검출과 결과 발행을 수행한다."""
        if self.busy:
            return
        self.busy = True
        try:
            if not self.first_frame_logged:
                self.get_logger().info(
                    "First camera frame received; "
                    "starting YOLO warm-up/inference"
                )
                self.first_frame_logged = True
            result = self.pipeline.predict(frame)
            confirmed = self.confirmation.update(bool(result.fallen))
            self.processed_frames += 1
            if self.processed_frames == 1:
                self.get_logger().info(
                    f"First inference complete: {result.inference_ms:.1f} ms; "
                    f"show_window={self.show_window}"
                )
            self._publish_outputs(
                source,
                result.fallen,
                result.helpers,
                confirmed,
                result.person_count,
                result.crowd_level,
                result.inference_ms,
            )
            publish_debug = bool(
                self.get_parameter("publish_debug_image").value
            )
            if publish_debug or self.show_window:
                self._publish_debug(source, result)
        except Exception as error:  # 카메라 콜백 자체가 죽지 않도록 로그 후 복구
            self.get_logger().error(f"Vision inference failed: {error}")
        finally:
            self.busy = False

    def _publish_outputs(
        self,
        source: CompressedImage,
        fallen: list[Box],
        helpers: list[Box],
        confirmed: bool,
        person_count: int,
        crowd_level: str,
        inference_ms: float,
    ) -> None:
        """프레임 상태를 발행하고 확정 상태의 상승/하강 에지를 이벤트로 만든다.

        status는 매 처리 프레임 발행하지만 EmergencyEvent는 False→True일 때
        CONFIRMED, True→False일 때 CANCELED를 한 번만 발행한다.
        """
        self.person_count_pub.publish(UInt32(data=person_count))
        self.crowd_pub.publish(String(data=crowd_level))
        payload = {
            "camera_id": self.camera_id,
            "zone_id": self.zone_id,
            "mode": self.mode,
            "fallen_detected": bool(fallen),
            "fallen_confirmed": confirmed,
            "fallen_count": len(fallen),
            "fallen_max_confidence": max(
                (box.confidence for box in fallen), default=0.0
            ),
            "helper_count": len(helpers),
            "person_count": person_count,
            "crowd_level": crowd_level,
            "confirmation_hits": self.confirmation.hit_count,
            "inference_ms": round(inference_ms, 2),
        }
        self.status_pub.publish(String(data=json.dumps(payload)))
        # 새 사고마다 고유 ID를 만들고 해제 이벤트까지 같은 ID를 유지한다.
        if confirmed and not self.was_confirmed:
            self.event_id = f"{self.camera_id}-{uuid4().hex[:12]}"
            self._publish_event(
                source, EmergencyEvent.CONFIRMED, fallen, self.event_id
            )
        elif not confirmed and self.was_confirmed:
            self._publish_event(
                source, EmergencyEvent.CANCELED, fallen, self.event_id
            )
            self.event_id = ""
        self.was_confirmed = confirmed

    def _publish_event(
        self,
        source: CompressedImage,
        status: int,
        fallen: list[Box],
        event_id: str,
    ) -> None:
        """기존 EmergencyEvent 형식으로 카메라 기반 응급 이벤트를 발행한다.

        카메라가 고정되어 있으므로 YAML의 대표 map 좌표를 사용한다.
        """
        event = EmergencyEvent()
        event.event_id = event_id
        event.detected_at = source.header.stamp
        event.location.header.stamp = source.header.stamp
        event.location.header.frame_id = self.frame_id
        event.location.point.x = self.location_x
        event.location.point.y = self.location_y
        event.location.point.z = 0.0
        event.confidence = max(
            (box.confidence for box in fallen), default=0.0
        )
        event.consecutive_detections = self.confirmation.hit_count
        event.status = status
        event.source_id = self.get_name()
        event.camera_id = self.camera_id
        event.zone_id = self.zone_id
        self.event_pub.publish(event)

    def _publish_debug(
        self,
        source: CompressedImage,
        result: InferenceOutput,
    ) -> None:
        """추론 결과를 로컬 창과 압축 디버그 토픽으로 출력한다."""
        debug = self.pipeline.render_debug(result, self.camera_id)
        if self.show_window:
            cv2.imshow(self.window_name, debug)
            cv2.waitKey(1)
        if not bool(self.get_parameter("publish_debug_image").value):
            return
        quality = int(self.get_parameter("debug_jpeg_quality").value)
        ok, encoded = cv2.imencode(
            ".jpg", debug, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if ok:
            output = CompressedImage()
            output.header = source.header
            output.format = "jpeg"
            output.data = encoded.tobytes()
            self.debug_pub.publish(output)

    def _publish_heartbeat(self) -> None:
        """영상 검출과 별도로 초당 한 번 노드 생존 신호와 순번을 발행한다."""
        self.sequence += 1
        message = Heartbeat()
        message.sender_id = f"aed_vision:{self.camera_id}"
        message.stamp = self.get_clock().now().to_msg()
        message.sequence = self.sequence
        self.heartbeat_pub.publish(message)

    def destroy_node(self) -> None:
        """노드 종료 시 이 노드가 생성한 OpenCV 결과 창을 닫는다."""
        if self.camera_source is not None:
            self.camera_source.close()
        if self.show_window:
            try:
                cv2.destroyWindow(self.window_name)
                cv2.waitKey(1)
            except cv2.error:
                # 첫 프레임 이전 종료라 창이 생성되지 않은 경우는 무시한다.
                pass
        super().destroy_node()


def main(args=None) -> None:
    """ROS2 context와 VisionDetector를 만들고 Ctrl+C까지 spin한다.

    finally에서 노드를 파괴하고 context를 종료해 publisher와 모델 자원이
    정상적으로 정리되도록 한다.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = VisionDetector()
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
