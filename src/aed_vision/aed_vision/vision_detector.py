"""AED 비전 패키지의 시작점이자 ROS 통신을 담당하는 조정자 노드.

전체 실행 순서는 다음과 같다.

1. YAML에서 들어온 ROS 파라미터를 읽고 모델 파이프라인을 한 번 생성한다.
2. USB 카메라를 직접 읽거나 ROS Image/CompressedImage 토픽을 구독한다.
3. 프레임을 ``InferencePipeline.predict``에 넘겨 단일 프레임 후보를 받는다.
4. 최근 프레임 기록으로 낙상을 확정하고, 필요하면 호모그래피 위치를 계산한다.
5. 상태·혼잡도·응급 이벤트·디버그 영상을 각각의 ROS 토픽에 발행한다.

모델 내부 판정은 ``inference_pipeline``에 있고, 이 파일은 입력과 출력 및 상태
전환을 연결한다. 따라서 코드 흐름을 공부할 때 ``_process_frame``부터 보면 된다.
"""

from __future__ import annotations

import json
import os
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from aed_interfaces.msg import (
    CrowdLevel,
    DetectionSummary,
    EmergencyEvent,
    Heartbeat,
)
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String, UInt32

from .camera_source import DirectCameraSource
from .detection_logic import (
    Box,
    CrowdStateStabilizer,
    TemporalConfirmation,
    crowd_metrics,
    update_presence_confirmation,
)
from .homography import Homography
from .inference_pipeline import InferenceOutput, InferencePipeline
from .qos import CAMERA_QOS


PARAMETER_DEFAULTS = (
    # ROS 2는 노드가 파라미터 이름과 자료형을 먼저 선언해야 YAML 값을 받을 수
    # 있다. 아래 값은 YAML이 없을 때의 안전 기본값이며 실제 배치값은 config의
    # 카메라별 YAML이 덮어쓴다. 즉 설정의 주 저장소는 YAML, 여기는 선언 목록이다.
    # 카메라 역할과 입력
    ("camera_id", "camera_open"),
    ("zone_id", "open_zone"),
    ("mode", "open"),
    ("image_topic", "/camera/image_raw/compressed"),
    ("image_is_compressed", True),
    ("direct_camera", True),
    ("camera_device", "auto"),
    ("width", 640),
    ("height", 480),
    ("fps", 15.0),
    ("frame_id", "aed_camera_optical_frame"),
    ("jpeg_quality", 85),
    # 모델과 추론
    ("detection_backend", "person_pose"),
    ("rescue_weights", ""),
    ("person_weights", ""),
    ("pose_weights", ""),
    ("rescue_conf", 0.25),
    ("person_conf", 0.5),
    ("pose_keypoint_conf", 0.3),
    ("pose_min_keypoints", 8),
    ("pose_min_box_area", 0.02),
    ("pose_min_torso_keypoints", 3),
    ("mannequin_bbox_fallback", True),
    ("mannequin_fallen_aspect_threshold", 1.03),
    ("posture_classifier_weights", ""),
    ("detect_people_as_helpers", False),
    ("iou", 0.5),
    ("imgsz", 640),
    ("inference_device", ""),
    # 검출 확정과 혼잡도
    ("confirmation_window", 10),
    ("confirmation_hits", 6),
    ("helper_confirmation_window", 6),
    ("helper_confirmation_hits", 3),
    ("helper_max_distance_ratio", 0.30),
    ("crowd_roi", [0.0, 0.0, 1.0, 1.0]),
    ("fallen_person_overlap_iou", 0.4),
    ("crowd_worsening_window", 5),
    ("crowd_worsening_hits", 3),
    ("crowd_improving_window", 10),
    ("crowd_improving_hits", 7),
    ("crowd_minimum_hold_seconds", 3.0),
    # map 좌표와 호모그래피
    ("location_frame_id", "map"),
    ("location_x", 0.0),
    ("location_y", 0.0),
    ("homography_camera_id", ""),
    ("homography_margin_m", 0.15),
    # 디버그 출력
    ("publish_debug_image", True),
    ("show_window", True),
    ("debug_jpeg_quality", 80),
)


def raw_image_to_bgr(message: Image) -> np.ndarray:
    """일반적인 8비트 ROS Image 메시지를 cv_bridge 없이 BGR 영상으로 변환한다.

    ROS Humble의 cv_bridge와 Ultralytics가 서로 다른 NumPy ABI를 사용할 수
    있으므로, 메시지의 바이트 버퍼를 직접 읽어 충돌 가능성을 피한다.
    """
    encoding = message.encoding.lower()
    channel_counts = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
        "8uc1": 1,
        "8uc3": 3,
        "8uc4": 4,
    }
    channels = channel_counts.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported raw image encoding: {message.encoding}")
    row_bytes = int(message.width) * channels
    step = int(message.step)
    if step < row_bytes:
        raise ValueError(
            f"invalid raw image step={step}; expected at least {row_bytes}"
        )
    expected_bytes = int(message.height) * step
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < expected_bytes:
        raise ValueError(
            f"short raw image buffer={raw.size}; expected {expected_bytes}"
        )
    pixels = raw[:expected_bytes].reshape(int(message.height), step)
    pixels = pixels[:, :row_bytes].reshape(
        int(message.height), int(message.width), channels
    )
    if encoding == "rgb8":
        return pixels[:, :, ::-1].copy()
    if encoding == "rgba8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    if encoding in ("bgra8", "8uc4"):
        return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    if channels == 1:
        return cv2.cvtColor(pixels[:, :, 0], cv2.COLOR_GRAY2BGR)
    return pixels.copy()


class VisionDetector(Node):
    """영상 입력, 프레임 확정 상태, ROS 결과 발행을 조율한다.

    open 모드는 구조 모델 하나만 실행하고, alley 모드는 같은 프레임에 구조
    모델과 COCO person 모델을 실행한다. 두 역할을 하나의 클래스로 구현해
    코드 중복을 막고 YAML의 ``mode``만으로 배치 장소를 구분한다.
    """

    def __init__(self) -> None:
        """설정→상태→모델→ROS 입출력 순서로 노드를 초기화한다."""
        super().__init__("vision_detector")
        self._declare_parameters()

        self._load_config()
        self._initialize_state()
        self._setup_window()
        self.pipeline = self._create_pipeline()
        self._create_publishers()
        self._setup_image_source()
        self.heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)
        self._log_configuration()

    def _param(self, name: str):
        """선언된 ROS 파라미터의 현재 값을 반환한다."""
        return self.get_parameter(name).value

    def _load_config(self) -> None:
        """ROS 파라미터를 검출기가 사용하는 설정값으로 변환한다."""
        self.camera_id = str(self._param("camera_id"))
        self.zone_id = str(self._param("zone_id"))
        self.mode = str(self._param("mode")).lower()
        if self.mode not in ("open", "alley", "robot"):
            raise ValueError("mode must be 'open', 'alley', or 'robot'")

        # 카메라 식별자는 토픽 경로와 이벤트 출처에 함께 사용한다.
        # 인파 모델은 연산량을 줄이기 위해 alley 모드에서만 메모리에 올린다.
        self.enable_crowd = self.mode == "alley"
        self.detect_people_as_helpers = bool(
            self._param("detect_people_as_helpers")
        )
        self.frame_id = str(self._param("location_frame_id"))
        self.location_x = float(self._param("location_x"))
        self.location_y = float(self._param("location_y"))
        homography_camera_id = str(self._param("homography_camera_id")).strip()
        self.homography = (
            Homography.load(camera_id=homography_camera_id)
            if homography_camera_id
            else None
        )
        self.homography_margin = float(self._param("homography_margin_m"))
        if self.homography_margin < 0.0:
            raise ValueError("homography_margin_m must be non-negative")
        self.crowd_roi = list(self._param("crowd_roi"))
        self.overlap_threshold = float(
            self._param("fallen_person_overlap_iou")
        )

    def _initialize_state(self) -> None:
        """시간 확정기와 프레임·이벤트 실행 상태를 초기화한다."""
        self.confirmation = TemporalConfirmation(
            int(self._param("confirmation_window")),
            int(self._param("confirmation_hits")),
        )
        self.helper_confirmation = TemporalConfirmation(
            int(self._param("helper_confirmation_window")),
            int(self._param("helper_confirmation_hits")),
        )
        self.crowd_stabilizer = CrowdStateStabilizer(
            int(self._param("crowd_worsening_window")),
            int(self._param("crowd_worsening_hits")),
            int(self._param("crowd_improving_window")),
            int(self._param("crowd_improving_hits")),
            float(self._param("crowd_minimum_hold_seconds")),
        )
        self.was_confirmed = False
        self.event_id = ""
        self.event_location = (self.location_x, self.location_y)
        self.event_detected_at = None
        self.event_confidence = 0.0
        self.event_confirmation_hits = 0
        self.event_crowd_level = CrowdLevel.NOT_APPLICABLE
        self.sequence = 0
        self.show_window = bool(self._param("show_window"))
        self.window_name = f"AED Vision - {self.camera_id} ({self.mode})"
        self.processed_frames = 0
        self.first_frame_logged = False
        self.busy = False

    def _setup_window(self) -> None:
        """요청된 경우 모델 로딩 상태를 보여줄 OpenCV 창을 만든다."""
        if not self.show_window:
            return
        has_display = os.environ.get("DISPLAY") or os.environ.get(
            "WAYLAND_DISPLAY"
        )
        if not has_display:
            self.get_logger().warning(
                "show_window=true but DISPLAY/WAYLAND_DISPLAY is not set; "
                "disabling the local window"
            )
            self.show_window = False
            return

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

    def _create_pipeline(self) -> InferencePipeline:
        """현재 설정에 맞는 추론 파이프라인을 생성한다."""
        return InferencePipeline(
            rescue_weights=str(self._param("rescue_weights")),
            person_weights=str(self._param("person_weights")),
            pose_weights=str(self._param("pose_weights")),
            detection_backend=str(self._param("detection_backend")),
            enable_crowd=self.enable_crowd,
            detect_people_as_helpers=self.detect_people_as_helpers,
            rescue_conf=float(self._param("rescue_conf")),
            person_conf=float(self._param("person_conf")),
            iou=float(self._param("iou")),
            imgsz=int(self._param("imgsz")),
            device=str(self._param("inference_device")),
            crowd_roi=self.crowd_roi,
            overlap_threshold=self.overlap_threshold,
            helper_max_distance_ratio=float(
                self._param("helper_max_distance_ratio")
            ),
            pose_keypoint_conf=float(self._param("pose_keypoint_conf")),
            pose_min_keypoints=int(self._param("pose_min_keypoints")),
            pose_min_box_area=float(self._param("pose_min_box_area")),
            pose_min_torso_keypoints=int(
                self._param("pose_min_torso_keypoints")
            ),
            mannequin_bbox_fallback=bool(
                self._param("mannequin_bbox_fallback")
            ),
            mannequin_fallen_aspect_threshold=float(
                self._param("mannequin_fallen_aspect_threshold")
            ),
            posture_classifier_weights=str(
                self._param("posture_classifier_weights")
            ),
        )

    def _create_publishers(self) -> None:
        """카메라별 namespace 아래에 결과 토픽 publisher를 생성한다.

        status/summary/count는 현재 관측값이고, emergency_event는 확정 상태가
        바뀔 때만 나가는 사건 메시지다. debug는 사람이 확인하는 JPEG 영상이다.
        """
        prefix = f"/{self.camera_id}/vision"
        self.event_pub = self.create_publisher(
            EmergencyEvent, f"{prefix}/emergency_event", 10
        )
        self.status_pub = self.create_publisher(String, f"{prefix}/status", 10)
        self.crowd_pub = self.create_publisher(
            CrowdLevel, f"{prefix}/crowd_level", 10
        )
        self.summary_pub = self.create_publisher(
            DetectionSummary, f"{prefix}/detection_summary", 10
        )
        self.person_count_pub = self.create_publisher(
            UInt32, f"{prefix}/person_count", 10
        )
        self.helper_count_pub = self.create_publisher(
            UInt32, f"{prefix}/helper_count", 10
        )
        self.helper_confirmed_pub = self.create_publisher(
            Bool, f"{prefix}/helper_confirmed", 10
        )
        self.fallen_location_pub = self.create_publisher(
            PointStamped, f"{prefix}/fallen_location", 10
        )
        self.heartbeat_pub = self.create_publisher(
            Heartbeat, f"{prefix}/heartbeat", 10
        )
        self.debug_pub = self.create_publisher(
            CompressedImage, f"{prefix}/debug/compressed", CAMERA_QOS
        )

    def _setup_image_source(self) -> None:
        """설정에 따라 ROS 토픽 구독과 USB 직접 입력 중 하나만 활성화한다."""
        self.image_topic = str(self._param("image_topic"))
        self.image_is_compressed = bool(self._param("image_is_compressed"))
        if self.image_is_compressed:
            self.subscription = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self._on_compressed_image,
                CAMERA_QOS,
            )
        else:
            self.subscription = self.create_subscription(
                Image, self.image_topic, self._on_raw_image, CAMERA_QOS
            )
        self.direct_camera = bool(self._param("direct_camera"))
        self.camera_source = None
        if self.direct_camera:
            # 같은 카메라를 구독과 직접 열기로 중복 처리하지 않도록, 먼저 만든
            # subscription을 제거하고 DirectCameraSource 콜백만 사용한다.
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.camera_source = DirectCameraSource(
                self, self.image_topic, self._process_frame
            )

    def _log_configuration(self) -> None:
        """시작 시 실제 적용된 주요 설정을 한 줄로 기록한다."""
        self.get_logger().info(
            f"camera={self.camera_id} mode={self.mode} "
            f"image={self.image_topic} "
            f"crowd_detection={self.enable_crowd} "
            f"people_as_helpers={self.detect_people_as_helpers} "
            f"detection_backend={self.pipeline.detection_backend} "
            f"input={'compressed' if self.image_is_compressed else 'raw'}"
        )

    def _declare_parameters(self) -> None:
        """YAML로 덮어쓸 수 있는 ROS 파라미터를 일괄 선언한다."""
        self.declare_parameters("", PARAMETER_DEFAULTS)

    def _on_compressed_image(self, message: CompressedImage) -> None:
        """구독한 JPEG 프레임을 디코딩해 추론한다."""
        frame = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            self.get_logger().warning("Failed to decode compressed image")
            return
        self._process_frame(frame, message)

    def _on_raw_image(self, message: Image) -> None:
        """OAK-D preview Image를 BGR OpenCV 프레임으로 바꿔 추론한다."""
        try:
            frame = raw_image_to_bgr(message)
        except ValueError as error:
            self.get_logger().warning(
                f"Failed to convert raw preview image: {error}"
            )
            return
        self._process_frame(frame, message)

    def _process_frame(
        self, frame: np.ndarray, source: CompressedImage | Image
    ) -> None:
        """디코딩된 한 프레임의 구조·혼잡 검출과 결과 발행을 수행한다."""
        # 핵심 흐름:
        # BGR 프레임 -> 모델 추론 -> 낙상 시간 확정 -> map 위치 계산
        # -> 프레임별 상태 발행 -> 필요할 때 디버그 영상 발행.
        # 이 함수는 각 세부 계산을 직접 하지 않고 전담 모듈을 조율한다.
        # 이전 프레임 추론이 아직 안 끝났으면 이번 프레임은 버린다.
        # 카메라 타이머/구독 콜백이 추론 속도보다 빠를 때 콜백이 쌓이는 것을 막는다
        # (콜백 큐잉 대신 여기서 직접 최신 프레임 우선 정책을 구현).
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
            # 1) 한 프레임의 낙상 후보, 조력자, 사람 수와 혼잡도를 계산한다.
            # 아직 이 결과는 단일 프레임의 후보이며 응급상황 확정은 아니다.
            result = self.pipeline.predict(frame)

            # 이번 프레임 단독 검출 여부(result.fallen)를 시간 창에 넣어
            # 순간 오검출을 걸러낸 "확정" 여부를 얻는다.
            confirmed = self.confirmation.update(bool(result.fallen))

            # 2) 현재 프레임에서 가장 신뢰도 높은 낙상 후보의 위치를 구한다.
            # 고정 카메라는 호모그래피, 미설정 카메라는 YAML 대표 좌표를 쓴다.
            target_location = self._target_location(result.fallen, frame.shape)

            # 측량 영역 밖(extrapolated)도 발행 대상에 포함— "configured"
            # (호모그래피 자체가 없는 경우)만 제외한다.
            if target_location[2].startswith("homography"):
                self._publish_fallen_location(
                    source, target_location[:2]
                )
            self.processed_frames += 1
            if self.processed_frames == 1:
                self.get_logger().info(
                    f"First inference complete: {result.inference_ms:.1f} ms; "
                    f"show_window={self.show_window}"
                )

            # 3) 프레임별 요약은 항상 발행하고, 응급 이벤트는 확정 상태가
            # 바뀐 순간에만 발행한다(False->True 또는 True->False).
            self._publish_outputs(
                source,
                result.fallen,
                result.pose_evidence,
                result.helpers,
                confirmed,
                result.person_count,
                result.crowd_level,
                result.inference_ms,
                target_location,
                result.crowd_time_multiplier,
                result.crowd_traversable,
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

    def _target_location(
        self, fallen: list[Box], frame_shape
    ) -> tuple[float, float, str]:
        """가장 확실한 쓰러진 사람의 bbox 하단 중심을 map 좌표로 변환한다."""
        # 반환값의 세 번째 항목은 좌표의 출처다.
        # configured: YAML 대표 좌표, homography: 측량 내부 변환,
        # homography_extrapolated: 측량 영역 밖으로 외삽한 변환.
        fallback = (self.location_x, self.location_y)
        if self.homography is None or not fallen:
            return fallback[0], fallback[1], "configured"
        target = max(fallen, key=lambda box: box.confidence)
        height, width = frame_shape[:2]
        x, y = self.homography.box_to_map(
            target.x1,
            target.y1,
            target.x2,
            target.y2,
            image_size=(width, height),
        )
        if not self.homography.inside_survey_area(
            x, y, margin=self.homography_margin
        ):
            self.get_logger().warning(
                "Detected location is outside surveyed area: "
                f"({x:.2f}, {y:.2f}); "
                "publishing extrapolated homography coordinates",
                throttle_duration_sec=5.0,
            )
            return x, y, "homography_extrapolated"
        return x, y, "homography"

    def _publish_fallen_location(
        self,
        source: CompressedImage | Image,
        location: tuple[float, float],
    ) -> None:
        """호모그래피로 계산한 검출 위치를 map 좌표 토픽으로 발행한다."""
        message = PointStamped()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self.frame_id
        message.point.x = location[0]
        message.point.y = location[1]
        message.point.z = 0.0
        self.fallen_location_pub.publish(message)

    def _publish_outputs(
        self,
        source: CompressedImage | Image,
        fallen: list[Box],
        pose_evidence: list,
        helpers: list[Box],
        confirmed: bool,
        person_count: int,
        crowd_level: int | None,
        inference_ms: float,
        target_location: tuple[float, float, str],
        crowd_time_multiplier: float | None,
        crowd_traversable: bool,
    ) -> None:
        """프레임 상태를 발행하고 확정 상태의 상승/하강 에지를 이벤트로 만든다.

        status는 매 처리 프레임 발행하지만 EmergencyEvent는 False→True일 때
        CONFIRMED, True→False일 때 CANCELED를 한 번만 발행한다.
        """
        # 이 함수의 출력은 두 종류다.
        # 1) 관측값: person/helper/crowd/status/summary를 매 프레임 발행
        # 2) 사건값: CONFIRMED/CANCELED를 상태 전환 때 한 번만 발행
        # person_count는 현재 프레임의 원본 관측값으로 발행한다. 반면 로봇의
        # 경로 결정을 바꾸는 혼잡 등급은 시간 안정화를 통과한 값으로 교체한다.
        observed_crowd_level = crowd_level
        if crowd_level is not None:
            crowd_level = self.crowd_stabilizer.update(crowd_level)
            (
                _stable_level,
                crowd_time_multiplier,
                crowd_traversable,
            ) = crowd_metrics(crowd_level)
        self.person_count_pub.publish(UInt32(data=person_count))
        helper_count = len(helpers)
        helper_confirmed = update_presence_confirmation(
            self.helper_confirmation, helper_count > 0
        )
        self.helper_count_pub.publish(UInt32(data=helper_count))
        self.helper_confirmed_pub.publish(Bool(data=helper_confirmed))
        crowd_message = CrowdLevel()
        crowd_message.camera_id = self.camera_id
        crowd_message.zone_id = self.zone_id
        crowd_message.stamp = source.header.stamp
        crowd_message.level = (
            CrowdLevel.NOT_APPLICABLE
            if crowd_level is None
            else crowd_level
        )
        crowd_message.person_count = person_count
        crowd_message.time_multiplier = (
            float("nan")
            if crowd_time_multiplier is None
            else crowd_time_multiplier
        )
        crowd_message.traversable = crowd_traversable
        self.crowd_pub.publish(crowd_message)
        payload = {
            "camera_id": self.camera_id,
            "zone_id": self.zone_id,
            "mode": self.mode,
            "detection_backend": self.pipeline.detection_backend,
            "fallen_detected": bool(fallen),
            "fallen_confirmed": confirmed,
            "fallen_count": len(fallen),
            "fallen_max_confidence": max(
                (box.confidence for box in fallen), default=0.0
            ),
            "helper_count": helper_count,
            "helper_confirmed": helper_confirmed,
            "helper_confirmation_hits": self.helper_confirmation.hit_count,
            "person_count": person_count,
            "crowd_level": crowd_level,
            "crowd_observed_level": observed_crowd_level,
            "crowd_time_multiplier": crowd_time_multiplier,
            "crowd_traversable": crowd_traversable,
            "confirmation_hits": self.confirmation.hit_count,
            "inference_ms": round(inference_ms, 2),
            "location_x": round(target_location[0], 3),
            "location_y": round(target_location[1], 3),
            "location_source": target_location[2],
        }
        fallen_pose = [
            evidence for evidence in pose_evidence
            if evidence.posture == "FALLEN"
        ]
        top_pose = max(
            fallen_pose or pose_evidence,
            key=lambda evidence: evidence.box.confidence,
            default=None,
        )
        payload.update(
            {
                "posture": top_pose.posture if top_pose else "",
                "pose_aspect_ratio": round(top_pose.aspect_ratio, 3)
                if top_pose else -1.0,
                "pose_torso_angle_deg": round(top_pose.torso_angle_deg, 2)
                if top_pose else -1.0,
                "pose_visible_keypoints": top_pose.visible_keypoints
                if top_pose else 0,
            }
        )
        self.status_pub.publish(String(data=json.dumps(payload)))
        self._publish_detection_summary(
            source, fallen, helpers, person_count, target_location
        )
        # was_confirmed와 비교해 상태가 "바뀐 프레임"에서만 이벤트를 쏜다.
        # 매 프레임 CONFIRMED를 반복 발행하면 구독 측(mission_manager 등)이
        # 매번 새 사고로 오인하므로, 상승 에지(False→True)/하강 에지(True→False)
        # 만 이벤트로 만든다.
        # 새 사고마다 고유 ID를 만들고 해제 이벤트까지 같은 ID를 유지한다.
        if confirmed and not self.was_confirmed:
            self.event_id = f"{self.camera_id}-{uuid4().hex[:12]}"
            self.event_location = target_location[:2]
            self.event_detected_at = source.header.stamp
            self.event_confidence = max(
                (box.confidence for box in fallen), default=0.0
            )
            self.event_confirmation_hits = self.confirmation.hit_count
            self.event_crowd_level = crowd_message.level
            self._publish_event(
                source,
                EmergencyEvent.CONFIRMED,
                self.event_id,
                self.event_location,
            )
        elif not confirmed and self.was_confirmed:
            self._publish_event(
                source,
                EmergencyEvent.CANCELED,
                self.event_id,
                self.event_location,
            )
            self.event_id = ""
            self.event_detected_at = None
        self.was_confirmed = confirmed

    def _publish_detection_summary(
        self,
        source: CompressedImage | Image,
        fallen: list[Box],
        helpers: list[Box],
        person_count: int,
        target_location: tuple[float, float, str],
    ) -> None:
        """문자열 JSON이 아닌 타입 고정 DetectionSummary를 매 프레임 발행한다."""
        summary = DetectionSummary()
        summary.camera_id = self.camera_id
        summary.stamp = source.header.stamp
        summary.person_count = person_count
        summary.fallen_count = len(fallen)
        summary.helper_count = len(helpers)
        summary.top_fallen_confidence = max(
            (box.confidence for box in fallen), default=0.0
        )
        summary.top_helper_confidence = max(
            (box.confidence for box in helpers), default=0.0
        )
        if fallen:
            summary.fallen_location.header.stamp = source.header.stamp
            summary.fallen_location.header.frame_id = self.frame_id
            summary.fallen_location.point.x = target_location[0]
            summary.fallen_location.point.y = target_location[1]
        self.summary_pub.publish(summary)

    def _publish_event(
        self,
        source: CompressedImage | Image,
        status: int,
        event_id: str,
        location: tuple[float, float],
    ) -> None:
        """확정 또는 취소 상태 전환을 EmergencyEvent 한 건으로 발행한다.

        고정 카메라는 호모그래피 좌표를 우선하고, 행렬이 없는 카메라는 YAML의
        대표 map 좌표를 사용한다. 확정과 취소는 같은 ``event_id``를 공유한다.
        """
        event = EmergencyEvent()
        event.event_id = event_id
        event.detected_at = self.event_detected_at or source.header.stamp
        event.location.header.stamp = event.detected_at
        event.location.header.frame_id = self.frame_id
        event.location.point.x = location[0]
        event.location.point.y = location[1]
        event.location.point.z = 0.0
        event.confidence = self.event_confidence
        event.consecutive_detections = self.event_confirmation_hits
        event.status = status
        event.source_id = self.get_name()
        event.camera_id = self.camera_id
        event.zone_id = self.zone_id
        event.crowd_level = self.event_crowd_level
        self.event_pub.publish(event)

    def _publish_debug(
        self,
        source: CompressedImage | Image,
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
