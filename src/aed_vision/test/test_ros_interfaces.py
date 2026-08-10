"""비전 노드가 채우는 공식 ROS 인터페이스의 회귀 테스트."""

from types import SimpleNamespace

from aed_interfaces.msg import CrowdLevel, DetectionSummary, EmergencyEvent
from builtin_interfaces.msg import Time

from aed_vision.detection_logic import Box
from aed_vision.vision_detector import (
    VisionDetector,
    helper_configuration_warning,
)


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def test_crowd_level_constants_match_four_stage_protocol() -> None:
    assert (
        CrowdLevel.CLEAR,
        CrowdLevel.BUSY,
        CrowdLevel.CROWDED,
        CrowdLevel.BLOCKED,
        CrowdLevel.NOT_APPLICABLE,
    ) == (0, 1, 2, 3, 255)


def test_robot_person_pose_warns_when_helper_detection_is_disabled() -> None:
    assert helper_configuration_warning("robot", "person_pose", False)
    assert helper_configuration_warning("robot", "person_pose", True) is None
    assert helper_configuration_warning("alley", "person_pose", False) is None


def test_canceled_event_preserves_confirmation_evidence() -> None:
    detector = VisionDetector.__new__(VisionDetector)
    detector.event_pub = _Publisher()
    detector.event_detected_at = Time(sec=10)
    detector.event_confidence = 0.87
    detector.event_confirmation_hits = 4
    detector.event_crowd_level = CrowdLevel.CROWDED
    detector.event_location_source = "homography"
    detector.event_location_valid = True
    detector.frame_id = "map"
    detector.camera_id = "camera_alley"
    detector.zone_id = "alley_zone"
    detector.get_name = lambda: "vision_detector"
    source = SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=20)))

    detector._publish_event(
        source, EmergencyEvent.CANCELED, "event-1", (1.0, 2.0)
    )

    event = detector.event_pub.messages[0]
    assert event.detected_at.sec == 10
    assert event.confidence == 0.87
    assert event.consecutive_detections == 4
    assert event.crowd_level == CrowdLevel.CROWDED
    assert event.location_source == "homography"
    assert event.location_valid is True


def test_detection_summary_omits_location_without_fallen_target() -> None:
    detector = VisionDetector.__new__(VisionDetector)
    detector.summary_pub = _Publisher()
    detector.camera_id = "camera_open"
    detector.frame_id = "map"
    source = SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=1)))

    detector._publish_detection_summary(
        source, [], [Box(0, 0, 10, 10, 0.5)], 1, (3.0, 4.0, "configured")
    )

    summary: DetectionSummary = detector.summary_pub.messages[0]
    assert summary.helper_count == 1
    assert summary.fallen_count == 0
    assert summary.fallen_location.header.frame_id == ""
