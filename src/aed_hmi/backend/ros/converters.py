"""ROS 메시지를 도메인 타입으로 옮긴다. 이 파일만 양쪽을 다 안다.

변환을 노드 안에 섞어 두면, 필드 하나가 바뀔 때 콜백 전체를 읽어야 한다.
여기 모아 두면 .msg 와 나란히 놓고 대조할 수 있다.

속도는 RobotState.msg 에 없어서 이전 표본과의 차이로 계산한다. 그 상태를
가진 것이 SpeedEstimator 이고, 순수 계산이라 ROS 없이 시험할 수 있다.
"""

import math
from typing import Optional

from ..domain.enums import (
    event_status,
    mission_state,
    robot_availability,
    robot_role,
)
from ..domain.models import (
    EmergencyEventSnapshot,
    MissionEvent,
    Point2D,
    RobotSnapshot,
)


def ros_time_to_epoch(stamp) -> float:
    """builtin_interfaces/Time -> UTC epoch 초."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(orientation) -> float:
    """평면 주행이라 yaw 만 쓴다. roll/pitch 는 무시한다."""
    return math.degrees(math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    ))


def battery_percentage_100(value: float) -> float:
    """Accept either ROS fractions or already-normalized percent values."""
    percentage = float(value)
    if not math.isfinite(percentage):
        return -1.0
    if 0.0 <= percentage <= 1.0:
        return percentage * 100.0
    return percentage


class SpeedEstimator:
    """연속된 위치에서 속도를 낸다. 로봇마다 하나씩 둔다.

    RobotState 는 속도를 싣지 않는다. odom 을 따로 구독하면 로봇당 토픽이
    하나 더 늘고 네임스페이스 리매핑도 따라붙어서, 이미 받는 pose 로 낸다.
    """

    # 표본 간격이 이보다 짧으면 위치 잡음이 속도로 증폭된다.
    MIN_INTERVAL_S = 0.2
    # 이보다 오래 끊겼으면 그 사이 어디를 지났는지 모른다. 속도를 버린다.
    MAX_INTERVAL_S = 5.0

    def __init__(self) -> None:
        self._last: Optional[tuple[float, float, float]] = None

    def update(self, stamp: float, x: float, y: float) -> float:
        previous = self._last
        self._last = (stamp, x, y)
        if previous is None:
            return 0.0
        elapsed = stamp - previous[0]
        if elapsed < self.MIN_INTERVAL_S or elapsed > self.MAX_INTERVAL_S:
            return 0.0
        distance = math.hypot(x - previous[1], y - previous[2])
        return distance / elapsed


def to_robot_snapshot(
    message, speed_mps: float, now: float
) -> RobotSnapshot:
    position = message.pose.pose.position
    last_heartbeat = ros_time_to_epoch(message.last_heartbeat)
    return RobotSnapshot(
        robot_id=message.robot_id,
        stamp=ros_time_to_epoch(message.stamp),
        position=Point2D(position.x, position.y),
        yaw_deg=yaw_from_quaternion(message.pose.pose.orientation),
        battery_percentage=battery_percentage_100(
            message.battery_percentage
        ),
        availability=robot_availability(message.availability),
        role=robot_role(message.role),
        mission_id=message.mission_id,
        is_docked=bool(message.is_docked),
        network_ok=bool(message.network_ok),
        localization_ok=bool(message.localization_ok),
        nav2_ok=bool(message.nav2_ok),
        emergency_stop=bool(message.emergency_stop),
        path_valid=bool(message.path_valid),
        estimated_path_cost=float(message.estimated_path_cost),
        last_heartbeat=last_heartbeat,
        detail=message.detail,
        speed_mps=speed_mps,
        # 하트비트를 한 번도 못 받았으면 0 이 들어온다. 그때는 나이를 0 으로
        # 두어 "방금 받았다"로 오해하지 않게 한다.
        heartbeat_age_s=(now - last_heartbeat) if last_heartbeat > 0 else 0.0,
    )


def to_emergency_event(message) -> EmergencyEventSnapshot:
    return EmergencyEventSnapshot(
        event_id=message.event_id,
        detected_at=ros_time_to_epoch(message.detected_at),
        location=Point2D(message.location.point.x, message.location.point.y),
        frame_id=message.location.header.frame_id,
        confidence=float(message.confidence),
        consecutive_detections=int(message.consecutive_detections),
        status=event_status(message.status),
        source_id=message.source_id,
        camera_id=message.camera_id,
        zone_id=message.zone_id,
        crowd_level=int(message.crowd_level),
    )


def to_assignment(message) -> dict:
    """MissionAssignment.msg -> context.on_assignment 인자.

    목표 좌표가 실려 오는 유일한 메시지다. MissionStatus 에는 상태만 있다.
    """
    return {
        "mission_id": message.mission_id,
        "version": int(message.assignment_version),
        "event_id": message.event_id,
        "robot_id": message.robot_id,
        "role": robot_role(message.role).value,
        "target": Point2D(
            message.target.pose.position.x, message.target.pose.position.y
        ),
        "assigned_at": ros_time_to_epoch(message.assigned_at),
    }


def to_mission_event(message) -> MissionEvent:
    return MissionEvent(
        mission_id=message.mission_id,
        event_id=message.event_id,
        robot_id=message.robot_id,
        assignment_version=int(message.assignment_version),
        state=mission_state(message.status),
        stamp=ros_time_to_epoch(message.stamp),
        reason=message.reason,
    )
