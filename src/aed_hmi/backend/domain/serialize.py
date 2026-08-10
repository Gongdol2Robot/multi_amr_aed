"""도메인 타입을 JSON 으로 내보낼 dict 로 바꾼다.

dataclasses.asdict 를 쓰지 않는다. 그것은 Enum 을 그대로 남기고, 계산 속성
(response_seconds, healthy)을 빠뜨린다. 화면이 그 값을 다시 계산하게 되면
같은 규칙이 두 곳에 생긴다. 규칙은 백엔드에만 둔다.
"""

from dataclasses import asdict

from .models import (
    CrowdZoneSnapshot,
    EmergencyEventSnapshot,
    MissionEvent,
    MissionSummary,
    Point2D,
    RobotSnapshot,
    StreamHealth,
    SystemSnapshot,
)


def point(value: Point2D) -> dict:
    return {"x": round(value.x, 4), "y": round(value.y, 4)}


def robot(value: RobotSnapshot) -> dict:
    data = asdict(value)
    data["position"] = point(value.position)
    data["availability"] = value.availability.value
    data["role"] = value.role.value
    data["healthy"] = value.healthy
    data["lidar_ok"] = value.lidar_ok
    return data


def emergency_event(value: EmergencyEventSnapshot) -> dict:
    data = asdict(value)
    data["location"] = point(value.location)
    data["status"] = value.status.value
    return data


def mission_event(value: MissionEvent) -> dict:
    data = asdict(value)
    data["state"] = value.state.value
    return data


def mission_summary(value: MissionSummary) -> dict:
    data = asdict(value)
    data["target"] = point(value.target)
    data["final_state"] = value.final_state.value
    data["response_seconds"] = value.response_seconds
    # 도착 예상 시각. 화면이 매 초 다시 계산하지 않도록 서버가 확정해 준다.
    data["eta_at"] = value.eta_at
    data["initial_eta_at"] = value.initial_eta_at
    return data


def stream_health(value: StreamHealth) -> dict:
    return asdict(value)


def crowd_zone(value: CrowdZoneSnapshot) -> dict:
    data = asdict(value)
    data["polygon"] = [point(item) for item in value.polygon]
    return data


def system_snapshot(value: SystemSnapshot) -> dict:
    return {
        "stamp": value.stamp,
        "robots": [robot(item) for item in value.robots],
        "active_event": (
            emergency_event(value.active_event) if value.active_event else None
        ),
        "active_missions": [
            mission_summary(item) for item in value.active_missions
        ],
        "streams": [stream_health(item) for item in value.streams],
        "crowd_zone": (
            crowd_zone(value.crowd_zone) if value.crowd_zone else None
        ),
        "ros_connected": value.ros_connected,
    }
