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
    RobotRole,
)
from ..domain.models import (
    CrowdZoneSnapshot,
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
        # 재할당이나 복귀 전환으로 더 이상 현재 상태가 아닌 임무들. ROS에서
        # 늦은 cancel 결과가 생략돼도 활성 배너를 붙잡지 못하게 한다.
        self._superseded_missions: set[str] = set()
        # mission_id -> (신고 시각, 목표 좌표). 요약을 만들 때 쓴다.
        self._mission_meta: dict[str, tuple[float, Point2D]] = {}
        self._mission_roles: dict[str, RobotRole] = {}
        # Nav2가 목표를 수락했고 RobotState가 실제 dock 이탈을 보고한 최초
        # 시각. 목표 수락만으로 시작하면 dock 위에서도 배너 시간이 흐른다.
        self._mission_dispatched_at: dict[str, float] = {}
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
        self._crowd_zone: Optional[CrowdZoneSnapshot] = None

    # ------------------------------------------------------------------
    # 쓰기 (ROS 스레드)
    # ------------------------------------------------------------------

    def put_robot(self, robot: RobotSnapshot) -> None:
        with self._lock:
            self._robots[robot.robot_id] = robot
            self._mark_dispatch_started(robot)

    def _mark_dispatch_started(self, robot: RobotSnapshot) -> None:
        """Record physical dispatch once Nav2 and dock state agree.

        Caller must hold ``self._lock``. Backend receive time is intentional:
        robot clocks need not be perfectly synchronized with the HMI clock.
        """
        if (
            robot.is_docked
            or robot.role != RobotRole.AED_DELIVERY
            or not robot.network_ok
            or not robot.nav2_ok
            or robot.emergency_stop
        ):
            return
        for mission in self._missions.values():
            if (
                mission.robot_id == robot.robot_id
                and mission.state == MissionState.EN_ROUTE
            ):
                self._mission_dispatched_at.setdefault(
                    mission.mission_id, time.time()
                )

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

    def put_crowd_zone(self, crowd_zone: CrowdZoneSnapshot) -> None:
        with self._lock:
            self._crowd_zone = crowd_zone

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
            # 중앙 manager는 임무 중 새 요청을 무시한다. 이미 활성 임무가
            # 있으면 그 임무와 무관한 새 카메라 이벤트가 배너를 빼앗지 못하게
            # 한다. 해당 이벤트의 MissionStatus가 오면 put_mission이 전환한다.
            active_event_ids = {
                item.event_id
                for item in self._missions.values()
                if item.state not in TERMINAL_MISSION_STATES
                and item.mission_id not in self._superseded_missions
                and self._mission_roles.get(
                    item.mission_id, RobotRole.AED_DELIVERY
                ) == RobotRole.AED_DELIVERY
            }
            if not active_event_ids or event.event_id in active_event_ids:
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
            previous = self._missions.get(mission.mission_id)
            if (
                previous is not None
                and mission.state == MissionState.ASSIGNED
                and mission.assignment_version
                <= previous.assignment_version
            ):
                return
            if previous is not None and (
                mission.assignment_version < previous.assignment_version
                or (
                    mission.assignment_version == previous.assignment_version
                    and previous.state != MissionState.ASSIGNED
                    and mission.stamp < previous.stamp
                )
            ):
                return
            if mission.state == MissionState.ASSIGNED:
                self._superseded_missions.discard(mission.mission_id)
                failure_states = {
                    MissionState.BLOCKED,
                    MissionState.NETWORK_LOST,
                    MissionState.NAVIGATION_ERROR,
                    MissionState.RECOVERY_WAIT,
                }
                for current in self._missions.values():
                    if (
                        current.mission_id != mission.mission_id
                        and current.event_id == mission.event_id
                        and current.assignment_version
                        < mission.assignment_version
                        and (
                            current.robot_id == mission.robot_id
                            or current.state in failure_states
                        )
                    ):
                        self._superseded_missions.add(current.mission_id)
            self._missions[mission.mission_id] = mission
            if mission.state == MissionState.EN_ROUTE:
                robot = self._robots.get(mission.robot_id)
                if robot is not None:
                    self._mark_dispatch_started(robot)
            if mission.mission_id not in self._mission_meta:
                self._mission_meta[mission.mission_id] = (
                    mission.stamp, Point2D(0.0, 0.0)
                )
            if mission.state not in TERMINAL_MISSION_STATES:
                event = self._events.get(mission.event_id)
                role = self._mission_roles.get(mission.mission_id)
                if event is not None and role in (
                    None, RobotRole.AED_DELIVERY
                ):
                    self._event = event
                elif (
                    role is not None
                    and role != RobotRole.AED_DELIVERY
                    and self._event is not None
                    and self._event.event_id == mission.event_id
                    and not self._has_active_mission(mission.event_id)
                ):
                    self._event = None
                return

            # 주행 오류는 응급 상황의 종료가 아니다. 재할당을 기다리거나
            # 대체 로봇이 없다는 실패 상태를 배너에 계속 보여야 한다.
            if mission.state == MissionState.NAVIGATION_ERROR:
                event = self._events.get(mission.event_id)
                if (
                    event is not None
                    and self._mission_roles.get(
                        mission.mission_id, RobotRole.AED_DELIVERY
                    ) == RobotRole.AED_DELIVERY
                ):
                    self._event = event
                return

            # 운영자 좌표 신고는 별도 RESOLVED 이벤트를 발행하지 않는다.
            # 도착/완료/취소를 관련 임무의 종료 신호로 쓴다.
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
            and item.mission_id not in self._superseded_missions
            and self._mission_roles.get(
                item.mission_id, RobotRole.AED_DELIVERY
            ) == RobotRole.AED_DELIVERY
            for item in self._missions.values()
        )

    def set_mission_target(
        self, mission_id: str, called_at: float, target: Point2D,
        *, role: RobotRole = RobotRole.NONE,
        initial_eta_seconds: float | None = None,
        current_eta_seconds: float | None = None,
    ) -> None:
        with self._lock:
            self._mission_meta[mission_id] = (called_at, target)
            self._mission_roles[mission_id] = role
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
            active_missions = [
                mission
                for mission in self._missions.values()
                if mission.state not in TERMINAL_MISSION_STATES
                and mission.mission_id not in self._superseded_missions
            ]
            event = self._event
            visible_missions = list(active_missions)
            # 대체 임무가 아직 없다면 마지막 주행 오류를 숨기지 않는다.
            if event is not None and not any(
                item.event_id == event.event_id for item in active_missions
            ):
                failures = [
                    item
                    for item in self._missions.values()
                    if item.event_id == event.event_id
                    and item.state == MissionState.NAVIGATION_ERROR
                    and item.mission_id not in self._superseded_missions
                    and self._mission_roles.get(
                        item.mission_id, RobotRole.AED_DELIVERY
                    ) == RobotRole.AED_DELIVERY
                ]
                if failures:
                    visible_missions.append(
                        max(failures, key=lambda item: item.stamp)
                    )
            active = [
                self._summarize(mission, now)
                for mission in visible_missions
            ]
            crowd_zone = self._crowd_zone
        return SystemSnapshot(
            stamp=now,
            robots=robots,
            active_event=event,
            active_missions=sorted(active, key=lambda item: item.called_at),
            streams=streams,
            crowd_zone=crowd_zone,
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
            dispatched_at=self._mission_dispatched_at.get(
                mission.mission_id
            ),
            arrived_at=None,
            final_state=mission.state,
            assignment_version=mission.assignment_version,
            reassignment_count=max(mission.assignment_version - 1, 0),
            role=self._mission_roles.get(
                mission.mission_id, RobotRole.NONE
            ),
            failure_reasons=[mission.reason] if mission.reason else [],
        )
