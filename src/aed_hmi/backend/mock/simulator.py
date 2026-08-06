"""ROS 없이 화면을 돌리기 위한 가짜 상태 생성기.

로봇이 없거나 다른 사람이 쓰는 동안에도 화면을 만들고 시연할 수 있어야 한다.
실제 시나리오(신고 → 배정 → 이동 → 도착 → 복귀)를 그대로 흉내내므로,
화면이 모든 상태를 한 번씩 그리는지 확인하는 데도 쓴다.

실제 좌표를 쓴다. maps/map.yaml 로 만든 지도와 오늘 측량한 Dock 위치가
기준이라, 화면의 지도 배치가 실제와 어긋나지 않는다.
"""

import math
import random
import threading
import time

from ..domain.enums import EventStatus, MissionState, RobotAvailability, RobotRole
from ..domain.models import (
    EmergencyEventSnapshot,
    MissionEvent,
    Point2D,
    RobotSnapshot,
)

# tools/initpose.py 로 실측한 Dock 좌표. src/aed_bringup/config/dock_poses.yaml
DOCKS = {
    "robot1": Point2D(-0.576, 0.137),
    "robot2": Point2D(-0.047, 0.049),
}
# 웹캠이 보는 구역의 중심. homography_cam1/cam2.yaml 의 측량 영역에서 가져왔다.
SCENE_TARGETS = (Point2D(1.99, 2.30), Point2D(-2.44, 1.84))

TICK_S = 0.2
CRUISE_SPEED = 0.22          # nav2_aed.yaml 의 max_vel_x 와 맞춘다


class MockSimulator:
    """한 건의 출동 시나리오를 반복 재생한다."""

    def __init__(self, context) -> None:
        self._context = context
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sequence = 0

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name="mock-sim", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ------------------------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._play_one_scenario()
            except Exception:
                # 시뮬레이터가 죽어도 서버는 살아 있어야 한다.
                time.sleep(1.0)

    def _play_one_scenario(self) -> None:
        self._sequence += 1
        event_id = f"evt-{self._sequence:04d}"
        mission_id = f"{event_id}-aed"
        target = random.choice(SCENE_TARGETS)
        now = time.time()

        positions = dict(DOCKS)
        # 도킹 대기
        self._idle(positions, seconds=4.0)
        if self._stop.is_set():
            return

        # 1) 웹캠이 쓰러진 사람을 본다. 한 프레임으로 확정하지 않고 연속
        #    검출이 쌓여야 CONFIRMED 가 되는 실제 규칙을 그대로 흉내낸다.
        camera = "camera_open" if target.x > 0 else "camera_alley"
        source = "119-dispatch" if self._sequence % 2 else camera
        for count in range(1, 4):
            if self._stop.is_set():
                return
            self._context.on_event(EmergencyEventSnapshot(
                event_id=event_id, detected_at=now, location=target,
                frame_id="map", confidence=0.55 + 0.12 * count,
                consecutive_detections=count, status=EventStatus.DETECTED,
                source_id=source, camera_id=camera, zone_id="zone-a",
            ))
            self._idle(positions, seconds=0.8)
        self._context.on_event(EmergencyEventSnapshot(
            event_id=event_id, detected_at=now, location=target,
            frame_id="map", confidence=0.91, consecutive_detections=4,
            status=EventStatus.CONFIRMED,
            source_id=source, camera_id=camera, zone_id="zone-a",
        ))

        # 2) 비용이 낮은 로봇 선정 — mission_manager 의 규칙과 같다
        chosen = min(
            positions,
            key=lambda rid: math.hypot(
                positions[rid].x - target.x, positions[rid].y - target.y
            ),
        )
        self._context.on_assignment(
            mission_id, 1, event_id, chosen,
            RobotRole.AED_DELIVERY.value, target, now,
        )
        self._emit_mission(mission_id, event_id, chosen, MissionState.ASSIGNED)
        self._emit_mission(mission_id, event_id, chosen,
                           MissionState.DISPATCHING)

        # 3) 이동
        self._emit_mission(mission_id, event_id, chosen, MissionState.EN_ROUTE)
        arrived = self._drive(positions, chosen, target, mission_id)
        if self._stop.is_set():
            return

        if not arrived:
            # 4) 경로 장애 → 다른 로봇으로 재할당. 시나리오 2번에 해당한다.
            self._emit_mission(mission_id, event_id, chosen,
                               MissionState.BLOCKED, "경로가 반복 실패")
            other = next(rid for rid in positions if rid != chosen)
            self._context.on_assignment(
                mission_id, 2, event_id, other,
                RobotRole.AED_DELIVERY.value, target, time.time(),
            )
            self._emit_mission(mission_id, event_id, other,
                               MissionState.EN_ROUTE, version=2)
            self._drive(positions, other, target, mission_id, force=True)
            chosen = other

        self._emit_mission(mission_id, event_id, chosen, MissionState.ARRIVED)
        self._idle(positions, seconds=3.0)

        # 5) 복귀
        self._drive(positions, chosen, DOCKS[chosen], mission_id, force=True,
                    role=RobotRole.RETURN)
        self._emit_mission(mission_id, event_id, chosen,
                           MissionState.COMPLETED)
        self._context.on_event(EmergencyEventSnapshot(
            event_id=event_id, detected_at=now, location=target,
            frame_id="map", confidence=0.91, consecutive_detections=4,
            status=EventStatus.RESOLVED, source_id=camera,
            camera_id=camera, zone_id="zone-a",
        ))

    # ------------------------------------------------------------------

    def _emit_mission(
        self, mission_id: str, event_id: str, robot_id: str,
        state: MissionState, reason: str = "", version: int = 1,
    ) -> None:
        self._context.on_mission(MissionEvent(
            mission_id=mission_id, event_id=event_id, robot_id=robot_id,
            assignment_version=version, state=state, stamp=time.time(),
            reason=reason,
        ))

    def _idle(self, positions: dict, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end and not self._stop.is_set():
            for robot_id, position in positions.items():
                self._publish(robot_id, position, 0.0,
                              RobotAvailability.AVAILABLE, RobotRole.NONE,
                              docked=True)
            time.sleep(TICK_S)

    def _drive(
        self, positions: dict, robot_id: str, goal: Point2D, mission_id: str,
        force: bool = False, role: RobotRole = RobotRole.AED_DELIVERY,
    ) -> bool:
        """목표까지 직선으로 간다. force 가 아니면 가끔 실패를 만든다."""
        # 3번에 1번은 도중에 막히게 해서, 재할당 화면이 실제로 보이게 한다.
        fail_at = None if force else (
            random.uniform(0.3, 0.6) if random.random() < 0.34 else None
        )
        start = positions[robot_id]
        total = math.hypot(goal.x - start.x, goal.y - start.y)
        travelled = 0.0
        while travelled < total and not self._stop.is_set():
            travelled += CRUISE_SPEED * TICK_S
            ratio = min(travelled / total, 1.0) if total > 0 else 1.0
            if fail_at is not None and ratio >= fail_at:
                return False
            positions[robot_id] = Point2D(
                start.x + (goal.x - start.x) * ratio,
                start.y + (goal.y - start.y) * ratio,
            )
            for other_id, position in positions.items():
                moving = other_id == robot_id
                self._publish(
                    other_id, position,
                    CRUISE_SPEED if moving else 0.0,
                    RobotAvailability.BUSY if moving
                    else RobotAvailability.AVAILABLE,
                    role if moving else RobotRole.NONE,
                    docked=not moving and self._at_dock(other_id, position),
                    mission_id=mission_id if moving else "",
                )
            time.sleep(TICK_S)
        return True

    @staticmethod
    def _at_dock(robot_id: str, position: Point2D) -> bool:
        dock = DOCKS[robot_id]
        return math.hypot(position.x - dock.x, position.y - dock.y) < 0.15

    def _publish(
        self, robot_id: str, position: Point2D, speed: float,
        availability: RobotAvailability, role: RobotRole,
        docked: bool = False, mission_id: str = "",
    ) -> None:
        now = time.time()
        self._context.on_robot(RobotSnapshot(
            robot_id=robot_id, stamp=now, position=position,
            yaw_deg=(now * 20) % 360 if speed > 0 else 5.3,
            battery_percentage=max(35.0, 92.0 - (self._sequence * 1.5)),
            availability=availability, role=role, mission_id=mission_id,
            is_docked=docked, network_ok=True, localization_ok=True,
            nav2_ok=True, emergency_stop=False, path_valid=True,
            estimated_path_cost=round(random.uniform(1.0, 6.0), 2),
            last_heartbeat=now, detail="",
            speed_mps=round(speed, 3), heartbeat_age_s=0.0,
        ))
