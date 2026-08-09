"""지금 이 순간의 상태를 들고 있다. 화면 한 장을 그리는 데 필요한 전부.

ROS 는 사건이 생길 때만 메시지를 보내지만, 화면은 항상 "현재 상태"를 봐야
한다. 그 간극을 메우는 것이 이 클래스다. 마지막으로 받은 값을 로봇별로
쥐고 있다가 주기적으로 통째로 내보낸다.

ROS 스레드가 쓰고 asyncio 가 읽으므로 잠금이 필요하다. 구간을 짧게 잡아
ROS 콜백이 오래 막히지 않게 한다.
"""

import threading
import time
from typing import Optional

from ..domain.enums import (
    TERMINAL_MISSION_STATES,
    EventStatus,
    MissionState,
)
from ..domain.models import (
    EmergencyEventSnapshot,
    MissionEvent,
    MissionSummary,
    Point2D,
    RobotSnapshot,
    StreamHealth,
    SystemSnapshot,
)

# 이 시간 동안 갱신이 없으면 로봇이 끊긴 것으로 본다. RobotState 발행 주기의
# 몇 배로 잡는다. 너무 짧으면 한 번 걸렀을 때 바로 빨간불이 켜진다.
ROBOT_STALE_AFTER_S = 6.0

# 검출 표시를 유지할 시간. 이벤트는 계속 오지 않으므로 마지막 값을 잠깐
# 붙잡아 둔다. 너무 길면 이미 끝난 검출이 화면에 남는다.
DETECTION_HOLD_S = 3.0


class LiveState:
    """마지막으로 받은 값들을 모아 현재 상태를 만든다."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._robots: dict[str, RobotSnapshot] = {}
        self._event: Optional[EmergencyEventSnapshot] = None
        # 검출기가 CANCELED를 보내도 이미 출동한 임무가 있으면 이벤트 정보가
        # 필요하다. 이벤트별 마지막 유효(비종료) 표본을 보관한다.
        self._events: dict[str, EmergencyEventSnapshot] = {}
        # mission_id -> 그 임무에서 마지막으로 본 상태 전이
        self._missions: dict[str, MissionEvent] = {}
        # mission_id -> (신고 시각, 목표 좌표). 요약을 만들 때 쓴다.
        self._mission_meta: dict[str, tuple[float, Point2D]] = {}
        # 배정 순간 중앙제어가 계산한 ETA. 이후 실시간 계산과 달리 고정한다.
        self._mission_initial_eta: dict[str, float] = {}
        # 중앙제어가 보낸 (남은 초, 수신 시각). HMI는 별도 계산하지 않는다.
        self._mission_current_eta: dict[str, tuple[float, float]] = {}
        # sensor_recovery 토픽은 RobotState와 독립적으로 들어온다. 로봇 상태
        # 표본과 합치는 것은 snapshot을 만드는 순간 한 번만 한다.
        self._lidar_states: dict[str, str] = {}
        self._fallback_states: dict[str, str] = {}
        # camera_id -> (연속 검출 수, 마지막으로 들은 시각)
        self._detections: dict[str, tuple[int, float]] = {}

    # ------------------------------------------------------------------
    # 쓰기 (ROS 스레드)
    # ------------------------------------------------------------------

    def put_robot(self, robot: RobotSnapshot) -> None:
        with self._lock:
            self._robots[robot.robot_id] = robot

    def put_lidar_state(self, robot_id: str, state: str) -> None:
        normalized = state.strip().upper()
        if normalized not in {"STARTING", "ALIVE", "FAULT", "RECOVERING"}:
            normalized = "UNKNOWN"
        with self._lock:
            self._lidar_states[robot_id] = normalized

    def put_fallback_state(self, robot_id: str, state: str) -> None:
        normalized = state.strip().upper()
        if normalized not in {
            "IDLE", "STARTING", "ACTIVE", "BLOCKED", "SUCCEEDED", "FAILED",
        }:
            normalized = "UNKNOWN"
        with self._lock:
            self._fallback_states[robot_id] = normalized

    def put_event(self, event: EmergencyEventSnapshot) -> None:
        with self._lock:
            # 어느 카메라가 무엇을 보고 있는지는 이벤트가 알려준다.
            # EmergencyEvent 가 camera_id 와 시간 창 confirmation hit 수를
            # 싣고 있어서, 검출 표시를 위해 별도 토픽을 만들 필요가 없다.
            if event.camera_id:
                self._detections[event.camera_id] = (
                    event.consecutive_detections, time.time()
                )
            # 카메라에서 대상이 사라졌다는 CANCELED는 검출 수명의 끝이지,
            # 이미 시작한 출동의 끝이 아니다. 관련 임무가 움직이는 동안은
            # 배너를 유지하고 MissionStatus가 끝났을 때 내린다.
            if event.status in (EventStatus.RESOLVED, EventStatus.CANCELED):
                if (
                    self._event
                    and self._event.event_id == event.event_id
                    and not self._has_active_mission(event.event_id)
                ):
                    self._event = None
                return
            self._events[event.event_id] = event
            self._event = event

    def put_person_count(self, camera_id: str, count: int) -> None:
        """vision_detector 가 매 프레임 내는 사람 수.

        EmergencyEvent 는 검출이 확정될 즈음에만 오지만 이 값은 계속 온다.
        화면의 "지금 몇 명 보이나"는 이쪽이 맞다.
        """
        with self._lock:
            self._detections[camera_id] = (count, time.time())

    def detection_counts(self) -> dict[str, int]:
        """카메라별 최근 검출 수. 오래된 것은 0 으로 떨어뜨린다.

        검출이 멈춘 뒤에도 마지막 숫자가 남아 있으면, 화면은 계속 무언가
        보고 있는 것처럼 보인다.
        """
        now = time.time()
        with self._lock:
            return {
                camera_id: (count if now - stamp <= DETECTION_HOLD_S else 0)
                for camera_id, (count, stamp) in self._detections.items()
            }

    def put_mission(self, mission: MissionEvent) -> None:
        with self._lock:
            self._missions[mission.mission_id] = mission
            if mission.mission_id not in self._mission_meta:
                self._mission_meta[mission.mission_id] = (
                    mission.stamp, Point2D(0.0, 0.0)
                )
            if mission.state not in TERMINAL_MISSION_STATES:
                event = self._events.get(mission.event_id)
                if event is not None:
                    self._event = event
                return

            # 운영자 좌표 신고는 별도 RESOLVED 이벤트를 발행하지 않는다.
            # 마지막 관련 임무의 ARRIVED/CANCELED/오류 상태를 종료 신호로 쓴다.
            if (
                self._event
                and self._event.event_id == mission.event_id
                and not self._has_active_mission(mission.event_id)
            ):
                self._event = None

    def _has_active_mission(self, event_id: str) -> bool:
        """Return whether the event still has a non-terminal robot mission.

        Caller must hold ``self._lock``.
        """
        return any(
            item.event_id == event_id
            and item.state not in TERMINAL_MISSION_STATES
            for item in self._missions.values()
        )

    def set_mission_target(
        self, mission_id: str, called_at: float, target: Point2D,
        *, initial_eta_seconds: float | None = None,
        current_eta_seconds: float | None = None,
    ) -> None:
        with self._lock:
            self._mission_meta[mission_id] = (called_at, target)
            if initial_eta_seconds is not None:
                self._mission_initial_eta[mission_id] = initial_eta_seconds
            if current_eta_seconds is not None:
                self._mission_current_eta[mission_id] = (
                    current_eta_seconds, called_at
                )

    def set_mission_eta(
        self, mission_id: str, seconds: float | None, received_at: float
    ) -> None:
        """Store a central mission-manager ETA update without recalculation."""
        with self._lock:
            if seconds is None:
                self._mission_current_eta.pop(mission_id, None)
            else:
                self._mission_current_eta[mission_id] = (seconds, received_at)

    # ------------------------------------------------------------------
    # 읽기 (asyncio 스레드)
    # ------------------------------------------------------------------

    def snapshot(
        self, streams: list[StreamHealth], ros_connected: bool
    ) -> SystemSnapshot:
        now = time.time()
        with self._lock:
            robots = [
                self._with_freshness(robot, now)
                for robot in sorted(
                    self._robots.values(), key=lambda item: item.robot_id
                )
            ]
            active = [
                self._summarize(mission, now)
                for mission in self._missions.values()
                if mission.state not in TERMINAL_MISSION_STATES
            ]
            event = self._event
        return SystemSnapshot(
            stamp=now,
            robots=robots,
            active_event=event,
            active_missions=sorted(active, key=lambda item: item.called_at),
            streams=streams,
            ros_connected=ros_connected,
        )

    def _with_freshness(self, robot: RobotSnapshot, now: float) -> RobotSnapshot:
        """오래된 표본은 통신 이상으로 표시한다.

        RobotState 가 끊기면 마지막 값이 그대로 남아, 화면은 멀쩡해 보이는데
        실제로는 로봇이 죽어 있는 상태가 된다. 그 함정을 여기서 막는다.
        """
        from dataclasses import replace

        robot = replace(
            robot,
            lidar_state=self._lidar_states.get(robot.robot_id, "UNKNOWN"),
            fallback_state=self._fallback_states.get(
                robot.robot_id, "UNKNOWN"
            ),
        )
        age = now - robot.stamp
        if age <= ROBOT_STALE_AFTER_S:
            return replace(robot, heartbeat_age_s=now - robot.last_heartbeat
                           if robot.last_heartbeat > 0 else age)
        return replace(
            robot,
            network_ok=False,
            speed_mps=0.0,
            heartbeat_age_s=age,
            detail=f"{age:.0f}초째 상태 수신 없음",
        )

    def _summarize(self, mission: MissionEvent, now: float) -> MissionSummary:
        called_at, target = self._mission_meta.get(
            mission.mission_id, (mission.stamp, Point2D(0.0, 0.0))
        )
        eta_update = self._mission_current_eta.get(mission.mission_id)
        remaining_eta = None
        if (
            eta_update is not None
            and mission.state in (
                MissionState.DISPATCHING, MissionState.EN_ROUTE,
            )
        ):
            remaining_eta = max(
                eta_update[0] - (now - eta_update[1]), 0.0
            )
        return MissionSummary(
            eta_seconds=remaining_eta,
            initial_eta_seconds=self._mission_initial_eta.get(
                mission.mission_id
            ),
            mission_id=mission.mission_id,
            event_id=mission.event_id,
            robot_id=mission.robot_id,
            target=target,
            called_at=called_at,
            dispatched_at=None,
            arrived_at=None,
            final_state=mission.state,
            assignment_version=mission.assignment_version,
            reassignment_count=max(mission.assignment_version - 1, 0),
            failure_reasons=[mission.reason] if mission.reason else [],
        )
