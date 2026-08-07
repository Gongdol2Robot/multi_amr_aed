"""aed_interfaces 의 uint8 상수를 사람이 읽는 이름으로 옮긴다.

메시지 정의에 있는 숫자를 화면까지 그대로 들고 가면, 프론트엔드가
`status === 7` 같은 코드를 쓰게 되고 의미가 사라진다. 변환은 여기 한 곳에서만
한다. .msg 파일이 바뀌면 이 파일만 고치면 된다.

이름은 .msg 의 상수 이름을 그대로 소문자로 옮긴 것이다. 임의로 짓지 않는다.
"""

from enum import Enum


class MissionState(str, Enum):
    """MissionStatus.msg 의 상태값."""

    ASSIGNED = "assigned"
    DISPATCHING = "dispatching"
    EN_ROUTE = "en_route"
    ARRIVED = "arrived"
    COMPLETED = "completed"
    CANCELED = "canceled"
    BLOCKED = "blocked"
    NETWORK_LOST = "network_lost"
    NAVIGATION_ERROR = "navigation_error"
    RECOVERY_WAIT = "recovery_wait"
    RECOVERY_RESUMED = "recovery_resumed"
    HELPER_REQUESTED = "helper_requested"
    HELPER_EN_ROUTE = "helper_en_route"
    HELPER_ARRIVED = "helper_arrived"


class RobotAvailability(str, Enum):
    """RobotState.msg 의 availability."""

    AVAILABLE = "available"
    BUSY = "busy"
    BLOCKED = "blocked"
    NETWORK_LOST = "network_lost"
    NAVIGATION_ERROR = "navigation_error"
    LOCALIZATION_ERROR = "localization_error"
    LOW_BATTERY = "low_battery"
    EMERGENCY_STOP = "emergency_stop"
    UNAVAILABLE = "unavailable"


class RobotRole(str, Enum):
    """RobotState.msg 와 MissionAssignment.msg 의 role."""

    NONE = "none"
    AED_DELIVERY = "aed_delivery"
    HELPER_REQUEST = "helper_request"
    GUIDE = "guide"
    RETURN = "return"


class EventStatus(str, Enum):
    """EmergencyEvent.msg 의 status."""

    DETECTED = "detected"
    CONFIRMED = "confirmed"
    DISPATCHED = "dispatched"
    RESOLVED = "resolved"
    CANCELED = "canceled"


# .msg 의 상수 선언 순서가 곧 정수값이다. 순서를 바꾸면 안 된다.
MISSION_STATE_BY_VALUE = {
    index: state for index, state in enumerate(MissionState)
}
ROBOT_AVAILABILITY_BY_VALUE = {
    index: value for index, value in enumerate(RobotAvailability)
}
ROBOT_ROLE_BY_VALUE = {index: value for index, value in enumerate(RobotRole)}
EVENT_STATUS_BY_VALUE = {
    index: value for index, value in enumerate(EventStatus)
}


# 운영자가 즉시 알아야 하는 상태. 화면에서 경고색으로 표시한다.
FAILURE_MISSION_STATES = frozenset({
    MissionState.BLOCKED,
    MissionState.NETWORK_LOST,
    MissionState.NAVIGATION_ERROR,
    MissionState.RECOVERY_WAIT,
})

TERMINAL_MISSION_STATES = frozenset({
    MissionState.ARRIVED,
    MissionState.COMPLETED,
    MissionState.CANCELED,
})


def mission_state(value: int) -> MissionState:
    """모르는 값이 오면 감추지 않고 드러낸다. 조용히 넘기면 원인을 못 찾는다."""
    try:
        return MISSION_STATE_BY_VALUE[value]
    except KeyError as error:
        raise ValueError(f"알 수 없는 MissionStatus.status: {value}") from error


def mission_state_from_name(name: str) -> MissionState:
    """저장소에서 읽은 문자열을 되돌린다. ROS 의 정수와는 다른 경로다."""
    try:
        return MissionState(name)
    except ValueError as error:
        raise ValueError(f"알 수 없는 임무 상태 이름: {name!r}") from error


def robot_availability(value: int) -> RobotAvailability:
    try:
        return ROBOT_AVAILABILITY_BY_VALUE[value]
    except KeyError as error:
        raise ValueError(
            f"알 수 없는 RobotState.availability: {value}"
        ) from error


def robot_role(value: int) -> RobotRole:
    try:
        return ROBOT_ROLE_BY_VALUE[value]
    except KeyError as error:
        raise ValueError(f"알 수 없는 role: {value}") from error


def event_status(value: int) -> EventStatus:
    try:
        return EVENT_STATUS_BY_VALUE[value]
    except KeyError as error:
        raise ValueError(
            f"알 수 없는 EmergencyEvent.status: {value}"
        ) from error
