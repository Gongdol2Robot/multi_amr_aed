"""비전 노드가 채우는 공식 ROS 인터페이스의 회귀 테스트."""

from types import SimpleNamespace
from time import monotonic

from aed_interfaces.msg import (
    CrowdLevel,
    DetectionSummary,
    EmergencyEvent,
    MissionAssignment,
    MissionStatus,
    RobotState,
)
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image

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


class _Logger:
    def __init__(self) -> None:
        self.info_messages = []
        self.warning_messages = []

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warning(self, message: str) -> None:
        self.warning_messages.append(message)


def _assignment_gated_detector():
    detector = VisionDetector.__new__(VisionDetector)
    detector.camera_id = "robot1"
    detector.wait_for_assignment = True
    detector.assignment_topic = "/robot1/mission_assignment"
    detector.mission_status_topic = "/aed/mission_status"
    parameters = {
        "image_topic": "/robot1/oakd/rgb/preview/image_raw",
        "image_is_compressed": False,
        "direct_camera": False,
    }
    detector._param = lambda name: parameters[name]
    detector.created_subscriptions = []
    detector.destroyed_subscriptions = []
    detector.confirmation = SimpleNamespace(clear=lambda: None)
    detector.helper_confirmation = SimpleNamespace(clear=lambda: None)
    detector.helper_confirmed_pub = _Publisher()
    detector.was_confirmed = False
    detector.event_id = ""
    detector.event_detected_at = None
    detector.first_frame_logged = True
    detector.last_successful_inference_at = 1.0
    detector.image_first_frame_timeout = 3.0
    detector.image_source_started_at = None
    detector.last_frame_received_at = None
    detector.image_restart_count = 0
    logger = _Logger()
    detector.get_logger = lambda: logger

    def create_subscription(message_type, topic, callback, qos):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        detector.created_subscriptions.append(subscription)
        return subscription

    detector.create_subscription = create_subscription
    detector.destroy_subscription = detector.destroyed_subscriptions.append
    return detector


def test_robot_image_subscription_waits_for_matching_assignment() -> None:
    detector = _assignment_gated_detector()
    detector._prepare_image_source()

    assert [item.topic for item in detector.created_subscriptions] == [
        "/robot1/mission_assignment",
        "/aed/mission_status",
    ]
    assert detector.subscription is None

    wrong_robot = MissionAssignment()
    wrong_robot.robot_id = "robot2"
    wrong_robot.mission_id = "event-1-aed-robot2"
    detector._on_mission_assignment(wrong_robot)
    assert detector.subscription is None

    assignment = MissionAssignment()
    assignment.robot_id = "robot1"
    assignment.mission_id = "event-1-aed-robot1"
    assignment.event_id = "event-1"
    assignment.assignment_version = 1
    assignment.role = RobotState.ROLE_AED_DELIVERY
    detector._on_mission_assignment(assignment)

    assert detector.subscription.message_type is Image
    assert detector.subscription.topic == (
        "/robot1/oakd/rgb/preview/image_raw"
    )


def test_duplicate_assignment_does_not_create_second_image_subscription() -> None:
    detector = _assignment_gated_detector()
    detector._prepare_image_source()
    assignment = MissionAssignment()
    assignment.robot_id = "robot1"
    assignment.mission_id = "event-1-aed-robot1"
    assignment.event_id = "event-1"
    assignment.assignment_version = 1
    assignment.role = RobotState.ROLE_AED_DELIVERY

    detector._on_mission_assignment(assignment)
    detector._on_mission_assignment(assignment)

    image_subscriptions = [
        item for item in detector.created_subscriptions
        if item.message_type is Image
    ]
    assert len(image_subscriptions) == 1


def test_reassignment_restarts_first_frame_monitoring() -> None:
    detector = _assignment_gated_detector()
    detector._prepare_image_source()
    assignment = MissionAssignment()
    assignment.robot_id = "robot1"
    assignment.mission_id = "event-1-aed-robot1"
    assignment.event_id = "event-1"
    assignment.assignment_version = 4
    assignment.role = RobotState.ROLE_AED_DELIVERY

    detector._on_mission_assignment(assignment)

    assert detector.first_frame_logged is False
    assert detector.last_successful_inference_at is None
    assert detector.image_source_started_at is not None
    assert detector.last_frame_received_at is None


def test_image_watchdog_recreates_subscription_without_losing_assignment() -> None:
    detector = _assignment_gated_detector()
    detector._prepare_image_source()
    assignment = MissionAssignment()
    assignment.robot_id = "robot1"
    assignment.mission_id = "event-1-aed-robot1"
    assignment.event_id = "event-1"
    assignment.assignment_version = 4
    assignment.role = RobotState.ROLE_AED_DELIVERY
    detector._on_mission_assignment(assignment)
    first_subscription = detector.subscription
    detector.image_source_started_at = monotonic() - 4.0

    detector._monitor_image_source()

    assert detector.subscription is not first_subscription
    assert first_subscription in detector.destroyed_subscriptions
    assert detector.active_event_id == "event-1"
    assert detector.active_assignment_version == 4
    assert detector.image_restart_count == 1
    assert detector.helper_confirmed_pub.messages[-1].data is False
    assert "no first frame" in detector.get_logger().warning_messages[-1]


def test_image_watchdog_stops_after_first_frame() -> None:
    detector = _assignment_gated_detector()
    detector._prepare_image_source()
    assignment = MissionAssignment()
    assignment.robot_id = "robot1"
    assignment.mission_id = "event-1-aed-robot1"
    assignment.event_id = "event-1"
    assignment.assignment_version = 1
    assignment.role = RobotState.ROLE_AED_DELIVERY
    detector._on_mission_assignment(assignment)
    subscription = detector.subscription
    # 첫 프레임 이후 시간이 오래 지났더라도 주행 중 구독에는 개입하지 않는다.
    detector.last_frame_received_at = monotonic() - 30.0

    detector._monitor_image_source()

    assert detector.subscription is subscription
    assert detector.image_restart_count == 0


def test_delivery_arrival_keeps_vision_until_helper_finishes() -> None:
    detector = _assignment_gated_detector()
    detector._prepare_image_source()
    assignment = MissionAssignment()
    assignment.robot_id = "robot1"
    assignment.mission_id = "event-1-aed-robot1"
    assignment.event_id = "event-1"
    assignment.assignment_version = 1
    assignment.role = RobotState.ROLE_AED_DELIVERY
    detector._on_mission_assignment(assignment)

    arrived = MissionStatus()
    arrived.robot_id = "robot1"
    arrived.mission_id = assignment.mission_id
    arrived.event_id = assignment.event_id
    arrived.assignment_version = assignment.assignment_version
    arrived.status = MissionStatus.ARRIVED
    detector._on_mission_status(arrived)
    assert detector.subscription is not None

    helper_finished = MissionStatus()
    helper_finished.robot_id = "robot1"
    helper_finished.mission_id = "event-1-helper-scan"
    helper_finished.event_id = "event-1"
    helper_finished.assignment_version = 2
    helper_finished.status = MissionStatus.HELPER_ARRIVED
    detector._on_mission_status(helper_finished)

    assert detector.subscription is None
    assert len(detector.destroyed_subscriptions) == 1
    assert detector.helper_confirmed_pub.messages[-1].data is False

    return_assignment = MissionAssignment()
    return_assignment.robot_id = "robot1"
    return_assignment.mission_id = "event-1-helper-return-robot1"
    return_assignment.event_id = "event-1"
    return_assignment.assignment_version = 3
    return_assignment.role = RobotState.ROLE_RETURN
    detector._on_mission_assignment(return_assignment)
    assert detector.subscription is not None


def test_return_arrival_deactivates_robot_image_subscription() -> None:
    detector = _assignment_gated_detector()
    detector._prepare_image_source()
    assignment = MissionAssignment()
    assignment.robot_id = "robot1"
    assignment.mission_id = "event-1-helper-return-robot1"
    assignment.event_id = "event-1"
    assignment.assignment_version = 3
    assignment.role = RobotState.ROLE_RETURN
    detector._on_mission_assignment(assignment)

    arrived = MissionStatus()
    arrived.robot_id = "robot1"
    arrived.mission_id = assignment.mission_id
    arrived.event_id = assignment.event_id
    arrived.assignment_version = assignment.assignment_version
    arrived.status = MissionStatus.ARRIVED
    detector._on_mission_status(arrived)

    assert detector.subscription is None


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
