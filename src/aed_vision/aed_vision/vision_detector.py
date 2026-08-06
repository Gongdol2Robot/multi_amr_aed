"""고정 카메라 영상에서 구조 대상과 선택적으로 골목 혼잡도를 검출한다."""

from __future__ import annotations

import json
import os
from pathlib import Path
from time import perf_counter
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from aed_interfaces.msg import EmergencyEvent, Heartbeat
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, UInt32

from .detection_logic import (
    Box,
    TemporalConfirmation,
    classify_crowd,
    count_crowd_people,
)
from .webcam_publisher import CAMERA_QOS


RESCUE_CLASS_NAMES = ["fallen_person", "helper_rc_car"]
# 학습 클래스명은 가중치 호환성 때문에 유지하고, 운영 화면만 helper로 표시한다.
DISPLAY_NAMES = {0: "fallen_person", 1: "helper"}


def _normalize_names(names: object) -> list[str]:
    """Ultralytics의 dict/list 클래스 이름을 ID 순서의 목록으로 바꾼다.

    모델 버전에 따라 ``names``가 {0: "..."} 또는 ["..."] 형식이므로
    비교 전에 통일한다. 클래스 순서가 바뀌면 bbox 의미도 바뀌기 때문에
    노드 시작 시 반드시 검증한다.
    """
    if isinstance(names, dict):
        return [str(names[key]) for key in sorted(names, key=int)]
    if isinstance(names, (list, tuple)):
        return [str(name) for name in names]
    raise RuntimeError(f"Unsupported model names: {names}")


def _required_path(value: str, description: str) -> Path:
    """모델 경로를 절대 경로로 바꾸고 파일 존재 여부를 검증한다.

    ``package://패키지명/상대경로``이면 설치된 ROS 패키지 share 폴더에서
    찾는다. 일반 경로에서는 ``$VAR``와 ``~``도 지원한다. 가중치가 없으면
    카메라 실행 전에 명확한 오류를 낸다.
    """
    package_prefix = "package://"
    if value.startswith(package_prefix):
        from ament_index_python.packages import get_package_share_directory

        package_path = value[len(package_prefix):]
        package_name, separator, relative_path = package_path.partition("/")
        if not separator or not package_name or not relative_path:
            raise RuntimeError(
                f"Invalid package model URI for {description}: {value}"
            )
        share_dir = Path(get_package_share_directory(package_name))
        path = (share_dir / relative_path).resolve()
    else:
        path = Path(os.path.expandvars(value)).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f"{description} not found: {path}")
    return path


def _boxes(result, class_id: int) -> list[Box]:
    """YOLO Result에서 한 클래스의 bbox와 confidence를 CPU 자료형으로 꺼낸다.

    후처리 계층이 torch Tensor와 Ultralytics API에 의존하지 않도록 모든 값을
    Python float 기반 ``Box``로 변환한다.
    """
    output = []
    if result.boxes is None:
        return output
    for xyxy, detected_class, confidence in zip(
        result.boxes.xyxy.cpu().tolist(),
        result.boxes.cls.int().cpu().tolist(),
        result.boxes.conf.cpu().tolist(),
    ):
        if detected_class == class_id:
            output.append(Box(*xyxy, confidence=float(confidence)))
    return output


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
        self.rescue_conf = float(self.get_parameter("rescue_conf").value)
        self.person_conf = float(self.get_parameter("person_conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.inference_device = str(
            self.get_parameter("inference_device").value
        )
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

        # import를 생성자 안에서 수행하면 모듈 설명이나 테스트 로직을 읽을 때
        # 무거운 torch/ultralytics 초기화를 피할 수 있다.
        from ultralytics import YOLO

        rescue_path = _required_path(
            str(self.get_parameter("rescue_weights").value),
            "rescue weights",
        )
        self.rescue_model = YOLO(str(rescue_path))
        rescue_names = _normalize_names(self.rescue_model.names)
        # 클래스 ID가 다르면 0/1 bbox를 완전히 반대로 처리하므로 즉시 중단한다.
        if rescue_names != RESCUE_CLASS_NAMES:
            raise RuntimeError(
                "Rescue classes must be "
                f"{RESCUE_CLASS_NAMES}, got {rescue_names}"
            )

        self.person_model = None
        self.person_class_id = -1
        if self.enable_crowd:
            person_path = _required_path(
                str(self.get_parameter("person_weights").value),
                "COCO person weights",
            )
            self.person_model = YOLO(str(person_path))
            person_names = _normalize_names(self.person_model.names)
            if "person" not in person_names:
                raise RuntimeError(
                    "The crowd model does not contain COCO person"
                )
            self.person_class_id = person_names.index("person")

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
        self.capture = None
        self.capture_timer = None
        self.image_pub = None
        if self.direct_camera:
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self._open_direct_camera(image_topic)
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

    def _open_direct_camera(self, image_topic: str) -> None:
        """이 노드가 직접 사용할 USB 카메라와 원본 영상 publisher를 준비한다."""
        device_value = str(self.get_parameter("camera_device").value)
        # "2"처럼 숫자만 지정한 경우에도 OpenCV 카메라 인덱스로 사용할 수 있다.
        device = int(device_value) if device_value.isdigit() else device_value
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open webcam: device={device}")
        self.capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")
        )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.image_pub = self.create_publisher(
            CompressedImage, image_topic, CAMERA_QOS
        )
        self.capture_timer = self.create_timer(
            1.0 / max(fps, 1.0), self._capture_and_infer
        )
        self.get_logger().info(
            f"Direct webcam {device}: {width}x{height}@{fps:.1f}"
        )

    def _capture_and_infer(self) -> None:
        """USB 카메라 한 프레임을 읽어 원본 토픽 발행 후 즉시 추론한다."""
        success, frame = self.capture.read()
        if not success:
            self.get_logger().warning("Failed to read direct webcam frame")
            return
        quality = int(self.get_parameter("jpeg_quality").value)
        encoded_ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if not encoded_ok:
            self.get_logger().warning("Failed to encode direct webcam frame")
            return
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.image_pub.publish(message)
        self._on_image(message)

    def _predict_options(self) -> dict:
        """구조/인파 모델에 동일하게 적용할 YOLO 추론 옵션을 반환한다.

        ``device``가 빈 문자열이면 Ultralytics가 GPU/CPU를 자동 선택한다.
        """
        options = {"iou": self.iou, "imgsz": self.imgsz, "verbose": False}
        if self.inference_device:
            options["device"] = self.inference_device
        return options

    def _on_image(self, message: CompressedImage) -> None:
        """압축 프레임 하나를 디코딩하고 역할별 전체 추론을 수행한다.

        alley 처리 순서는 구조 검출 → COCO person 검출 → ROI/중복 제거 →
        혼잡도 분류이다. 예외는 로그로 남기되 다음 프레임 콜백은 계속 받는다.
        """
        # SingleThreadedExecutor에서도 처리 지연 시 재진입 가능성을 방어하고,
        # 실시간성이 중요한 영상에서 오래된 프레임을 뒤늦게 처리하지 않는다.
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
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warning("Failed to decode compressed image")
                return
            started = perf_counter()
            rescue_result = self.rescue_model.predict(
                frame, conf=self.rescue_conf, **self._predict_options()
            )[0]
            fallen = _boxes(rescue_result, 0)
            helpers = _boxes(rescue_result, 1)
            # 한 프레임 검출을 바로 응급상황으로 만들지 않고 시간 창에 누적한다.
            confirmed = self.confirmation.update(bool(fallen))
            person_count = 0
            crowd_level = "NOT_APPLICABLE"
            person_result = None
            if self.enable_crowd and self.person_model is not None:
                # COCO의 80개 클래스 중 class 0 person만 추론해 불필요한 bbox와
                # 후처리 비용을 줄인다.
                person_result = self.person_model.predict(
                    frame,
                    conf=self.person_conf,
                    classes=[self.person_class_id],
                    **self._predict_options(),
                )[0]
                people = _boxes(person_result, self.person_class_id)
                height, width = frame.shape[:2]
                person_count = count_crowd_people(
                    people,
                    fallen,
                    (width, height),
                    self.crowd_roi,
                    self.overlap_threshold,
                )
                crowd_level = classify_crowd(
                    person_count, self.crowded_threshold
                )
            inference_ms = (perf_counter() - started) * 1000.0
            self.processed_frames += 1
            if self.processed_frames == 1:
                self.get_logger().info(
                    f"First inference complete: {inference_ms:.1f} ms; "
                    f"show_window={self.show_window}"
                )
            # 구조화된 이벤트와 모니터링 상태를 분리해 발행한다. 중앙 Mission
            # Manager는 이벤트를, 디버깅/HMI는 status 토픽을 사용할 수 있다.
            self._publish_outputs(
                message,
                fallen,
                helpers,
                confirmed,
                person_count,
                crowd_level,
                inference_ms,
            )
            publish_debug = bool(
                self.get_parameter("publish_debug_image").value
            )
            if publish_debug or self.show_window:
                self._publish_debug(
                    message,
                    rescue_result,
                    person_result,
                    crowd_level,
                    person_count,
                )
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

        카메라가 고정되어 있으므로 현재는 YAML의 대표 map 좌표를 사용한다.
        추후 Homography가 현장 측량되면 fallen bbox 하단 중앙점 좌표로 교체한다.
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
        rescue_result,
        person_result,
        crowd_level: str,
        person_count: int,
    ) -> None:
        """구조 bbox, COCO person bbox, 혼잡 ROI와 상태를 JPEG로 발행한다.

        구조 bbox는 모델 기본 색상, COCO person은 자홍색, 골목 ROI는 청록색을
        사용한다. 운영 네트워크 부하가 크면 publish_debug_image를 끌 수 있다.
        """
        rescue_result.names = DISPLAY_NAMES
        debug = rescue_result.plot()
        if person_result is not None and person_result.boxes is not None:
            # 결과 화면에서는 ROI 밖/중복 person도 보이게 해 현장 튜닝 근거를
            # 제공하되, 상단 people 수에는 필터를 통과한 bbox만 반영한다.
            for xyxy in person_result.boxes.xyxy.int().cpu().tolist():
                x1, y1, x2, y2 = xyxy
                cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 0, 255), 2)
        height, width = debug.shape[:2]
        x1 = int(self.crowd_roi[0] * width)
        y1 = int(self.crowd_roi[1] * height)
        x2 = int(self.crowd_roi[2] * width)
        y2 = int(self.crowd_roi[3] * height)
        if self.enable_crowd:
            cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 255, 0), 2)
        # open 카메라는 인파 판단 자체를 하지 않으므로 people=0이나
        # NOT_APPLICABLE을 화면에 표시하지 않는다. 혼잡 정보는 alley 전용이다.
        status_text = (
            f"{self.camera_id} | {crowd_level} | people={person_count}"
            if self.enable_crowd
            else f"{self.camera_id} | rescue detection"
        )
        cv2.putText(
            debug,
            status_text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        # ROS 토픽 구독 도구 없이도 각 카메라 노트북에서 즉시 결과를 확인한다.
        # waitKey가 GUI 이벤트를 처리하므로 imshow와 함께 반드시 호출해야 한다.
        if self.show_window:
            cv2.imshow(self.window_name, debug)
            cv2.waitKey(1)

        # 로컬 창만 필요한 경우 JPEG 인코딩과 ROS 네트워크 발행 비용을 생략한다.
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
        if self.capture is not None:
            self.capture.release()
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
