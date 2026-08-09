"""고정 카메라 영상에서 구조 대상과 선택적으로 골목 혼잡도를 검출한다."""

from __future__ import annotations

import json
import os
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from aed_interfaces.msg import EmergencyEvent, Heartbeat
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String, UInt32

from .camera_source import DirectCameraSource
from .detection_logic import (
    Box,
    TemporalConfirmation,
    update_presence_confirmation,
)
from .homography import Homography
from .inference_pipeline import InferenceOutput, InferencePipeline
from .qos import CAMERA_QOS


PARAMETER_DEFAULTS = (
    # 카메라 역할과 입력
    ("camera_id", "camera_open"),
    ("zone_id", "open_zone"),
    ("mode", "open"),
    ("image_topic", "/camera/image_raw/compressed"),
    ("image_is_compressed", True),
    ("direct_camera", True),
    ("camera_device", "/dev/video2"),
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
    ("crowded_person_threshold", 3),
    ("fallen_person_overlap_iou", 0.4),
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
    """Convert common 8-bit ROS Image encodings without cv_bridge.

    ROS Humble's cv_bridge extension is built against the system NumPy ABI,
    while Ultralytics may use a newer user-site NumPy.  Reading the byte
    buffer directly avoids loading incompatible NumPy ABIs in one process.
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
        if self.mode not in ("open", "alley", "robot"):
            raise ValueError("mode must be 'open', 'alley', or 'robot'")
        # 인파 모델은 연산량을 줄이기 위해 alley 모드에서만 메모리에 올린다.
        self.enable_crowd = self.mode == "alley"
        self.detect_people_as_helpers = bool(
            self.get_parameter("detect_people_as_helpers").value
        )
        self.frame_id = str(self.get_parameter("location_frame_id").value)
        self.location_x = float(self.get_parameter("location_x").value)
        self.location_y = float(self.get_parameter("location_y").value)
        homography_camera_id = str(
            self.get_parameter("homography_camera_id").value
        ).strip()
        self.homography = (
            Homography.load(camera_id=homography_camera_id)
            if homography_camera_id
            else None
        )
        # EmergencyEvent에 넣을 대표 위치. 호모그래피가 없으면 YAML의
        # 고정 location_x/y를 그대로 쓰고, 있으면 매 확정마다 갱신된다.
        self.event_location = (self.location_x, self.location_y)
        self.homography_margin = float(
            self.get_parameter("homography_margin_m").value
        )
        if self.homography_margin < 0.0:
            raise ValueError("homography_margin_m must be non-negative")
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
        helper_window = int(
            self.get_parameter("helper_confirmation_window").value
        )
        helper_hits = int(
            self.get_parameter("helper_confirmation_hits").value
        )
        self.helper_confirmation = TemporalConfirmation(
            helper_window, helper_hits
        )
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
            pose_weights=str(self.get_parameter("pose_weights").value),
            detection_backend=str(
                self.get_parameter("detection_backend").value
            ),
            enable_crowd=self.enable_crowd,
            detect_people_as_helpers=self.detect_people_as_helpers,
            rescue_conf=float(self.get_parameter("rescue_conf").value),
            person_conf=float(self.get_parameter("person_conf").value),
            iou=float(self.get_parameter("iou").value),
            imgsz=int(self.get_parameter("imgsz").value),
            device=str(self.get_parameter("inference_device").value),
            crowd_roi=self.crowd_roi,
            crowded_threshold=self.crowded_threshold,
            overlap_threshold=self.overlap_threshold,
            helper_max_distance_ratio=float(
                self.get_parameter("helper_max_distance_ratio").value
            ),
            pose_keypoint_conf=float(
                self.get_parameter("pose_keypoint_conf").value
            ),
            pose_min_keypoints=int(
                self.get_parameter("pose_min_keypoints").value
            ),
            pose_min_box_area=float(
                self.get_parameter("pose_min_box_area").value
            ),
            pose_min_torso_keypoints=int(
                self.get_parameter("pose_min_torso_keypoints").value
            ),
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
        image_topic = str(self.get_parameter("image_topic").value)
        self.image_is_compressed = bool(
            self.get_parameter("image_is_compressed").value
        )
        # 우선 구독을 만들어 두고, direct_camera 모드면 바로 아래에서 폐기한다.
        # (robot 모드처럼 direct_camera=False인 경우에는 이 구독을 그대로 쓴다.)
        if self.image_is_compressed:
            self.subscription = self.create_subscription(
                CompressedImage,
                image_topic,
                self._on_compressed_image,
                CAMERA_QOS,
            )
        else:
            self.subscription = self.create_subscription(
                Image, image_topic, self._on_raw_image, CAMERA_QOS
            )
        # 기본 실행은 한 노드가 웹캠을 직접 읽는다. 같은 노트북 안에서 영상을
        # DDS로 왕복시키지 않아 화면 지연과 publisher/subscriber 연결 문제를 없앤다.
        self.direct_camera = bool(
            self.get_parameter("direct_camera").value
        )
        self.camera_source = None
        if self.direct_camera:
            # 구독 대신 웹캠을 직접 폴링하는 DirectCameraSource로 대체한다.
            # image_topic은 그대로 두어 이 노드가 발행하는 원본 영상 토픽 이름으로
            # 재사용한다(구독은 안 하고 publish만).
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.camera_source = DirectCameraSource(
                self, image_topic, self._process_frame
            )
        # 영상 유무와 관계없이 중앙 시스템이 노드 생존을 판단할 수 있게 한다.
        self.heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)
        self.get_logger().info(
            f"camera={self.camera_id} mode={self.mode} image={image_topic} "
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
            result = self.pipeline.predict(frame)
            # 이번 프레임 단독 검출 여부(result.fallen)를 시간 창에 넣어
            # 순간 오검출을 걸러낸 "확정" 여부를 얻는다.
            confirmed = self.confirmation.update(bool(result.fallen))
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
        """가장 확실한 쓰러진 사람의 bbox 중심을 map 좌표로 변환한다."""
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
        self.person_count_pub.publish(UInt32(data=person_count))
        helper_count = len(helpers)
        helper_confirmed = update_presence_confirmation(
            self.helper_confirmation, helper_count > 0
        )
        self.helper_count_pub.publish(UInt32(data=helper_count))
        self.helper_confirmed_pub.publish(Bool(data=helper_confirmed))
        crowd_value = (
            "NOT_APPLICABLE" if crowd_level is None else str(crowd_level)
        )
        self.crowd_pub.publish(String(data=crowd_value))
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
        # was_confirmed와 비교해 상태가 "바뀐 프레임"에서만 이벤트를 쏜다.
        # 매 프레임 CONFIRMED를 반복 발행하면 구독 측(mission_manager 등)이
        # 매번 새 사고로 오인하므로, 상승 에지(False→True)/하강 에지(True→False)
        # 만 이벤트로 만든다.
        # 새 사고마다 고유 ID를 만들고 해제 이벤트까지 같은 ID를 유지한다.
        if confirmed and not self.was_confirmed:
            self.event_id = f"{self.camera_id}-{uuid4().hex[:12]}"
            self.event_location = target_location[:2]
            self._publish_event(
                source,
                EmergencyEvent.CONFIRMED,
                fallen,
                self.event_id,
                self.event_location,
            )
        elif not confirmed and self.was_confirmed:
            self._publish_event(
                source,
                EmergencyEvent.CANCELED,
                fallen,
                self.event_id,
                self.event_location,
            )
            self.event_id = ""
        self.was_confirmed = confirmed

    def _publish_event(
        self,
        source: CompressedImage | Image,
        status: int,
        fallen: list[Box],
        event_id: str,
        location: tuple[float, float],
    ) -> None:
        """기존 EmergencyEvent 형식으로 카메라 기반 응급 이벤트를 발행한다.

        카메라가 고정되어 있으므로 YAML의 대표 map 좌표를 사용한다.
        """
        event = EmergencyEvent()
        event.event_id = event_id
        event.detected_at = source.header.stamp
        event.location.header.stamp = source.header.stamp
        event.location.header.frame_id = self.frame_id
        event.location.point.x = location[0]
        event.location.point.y = location[1]
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
