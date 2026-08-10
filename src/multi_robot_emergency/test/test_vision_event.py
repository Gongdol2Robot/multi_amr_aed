from aed_interfaces.msg import EmergencyEvent

from multi_robot_emergency.mission_manager import EmergencyMissionManager


class _Logger:
    def __init__(self) -> None:
        self.errors = []
        self.infos = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def info(self, message: str) -> None:
        self.infos.append(message)


class _ManagerHarness:
    def __init__(self) -> None:
        self.processed_event_ids = set()
        self.requests = []
        self.logger = _Logger()

    def get_logger(self) -> _Logger:
        return self.logger

    def _on_request(self, request, request_id=None) -> None:
        self.requests.append((request, request_id))


def _event(*, status: int, location_valid: bool = True) -> EmergencyEvent:
    message = EmergencyEvent()
    message.event_id = "vision-event-001"
    message.camera_id = "camera_open"
    message.status = status
    message.location_valid = location_valid
    message.location_source = "homography"
    message.location.header.frame_id = "map"
    message.location.point.x = 1.25
    message.location.point.y = -0.75
    return message


def _handle(manager: _ManagerHarness, message: EmergencyEvent) -> None:
    EmergencyMissionManager._on_emergency_event(manager, message)


def test_vision_event_requires_confirmation() -> None:
    manager = _ManagerHarness()

    _handle(manager, _event(status=EmergencyEvent.DETECTED))

    assert manager.requests == []
    assert manager.processed_event_ids == set()


def test_vision_event_rejects_unsafe_location() -> None:
    manager = _ManagerHarness()

    _handle(
        manager,
        _event(status=EmergencyEvent.CONFIRMED, location_valid=False),
    )

    assert manager.requests == []
    assert manager.processed_event_ids == set()
    assert manager.logger.errors


def test_confirmed_vision_event_becomes_one_planning_request() -> None:
    manager = _ManagerHarness()
    message = _event(status=EmergencyEvent.CONFIRMED)

    _handle(manager, message)
    _handle(manager, message)

    assert len(manager.requests) == 1
    request, request_id = manager.requests[0]
    assert request_id == "vision-event-001"
    assert request.header.frame_id == "map"
    assert request.pose.position.x == 1.25
    assert request.pose.position.y == -0.75
    assert request.pose.orientation.w == 1.0
    assert manager.processed_event_ids == {"vision-event-001"}
