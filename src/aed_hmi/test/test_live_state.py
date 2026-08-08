import time

import pytest

from backend.domain.enums import EventStatus, MissionState
from backend.domain.models import (
    EmergencyEventSnapshot,
    MissionEvent,
    Point2D,
)
from backend.stream.live_state import LiveState
from backend.domain.enums import RobotAvailability, RobotRole
from backend.domain.models import RobotSnapshot


def event(event_id: str = "event-1") -> EmergencyEventSnapshot:
    return EmergencyEventSnapshot(
        event_id=event_id,
        detected_at=100.0,
        location=Point2D(1.0, 2.0),
        frame_id="map",
        confidence=0.9,
        consecutive_detections=3,
        status=EventStatus.CONFIRMED,
        source_id="vision",
        camera_id="camera_open",
        zone_id="open",
    )


def mission(state: MissionState, event_id: str = "event-1") -> MissionEvent:
    return MissionEvent(
        mission_id=f"{event_id}-aed-robot1",
        event_id=event_id,
        robot_id="robot1",
        assignment_version=1,
        state=state,
        stamp=101.0,
        reason="",
    )


def active_event(state: LiveState):
    return state.snapshot([], True).active_event


def test_camera_cancel_does_not_hide_an_active_dispatch() -> None:
    state = LiveState()
    confirmed = event()
    state.put_event(confirmed)
    state.put_mission(mission(MissionState.EN_ROUTE))
    state.put_event(EmergencyEventSnapshot(
        **{**confirmed.__dict__, "status": EventStatus.CANCELED}
    ))
    assert active_event(state) == confirmed


def test_arrival_clears_operator_event_without_resolved_message() -> None:
    state = LiveState()
    state.put_event(event("operator-1"))
    state.put_mission(mission(MissionState.EN_ROUTE, "operator-1"))
    state.put_mission(mission(MissionState.ARRIVED, "operator-1"))
    assert active_event(state) is None


def test_event_waits_for_all_related_missions_to_finish() -> None:
    state = LiveState()
    state.put_event(event())
    first = mission(MissionState.EN_ROUTE)
    second = MissionEvent(
        **{**first.__dict__, "mission_id": "event-1-aed-robot2",
           "robot_id": "robot2"}
    )
    state.put_mission(first)
    state.put_mission(second)
    state.put_mission(MissionEvent(
        **{**first.__dict__, "state": MissionState.ARRIVED}
    ))
    assert active_event(state) is not None
    state.put_mission(MissionEvent(
        **{**second.__dict__, "state": MissionState.CANCELED}
    ))
    assert active_event(state) is None


def test_initial_eta_is_frozen_on_the_mission_summary() -> None:
    state = LiveState()
    state.put_event(event())
    state.set_mission_target(
        "event-1-aed-robot1",
        101.0,
        Point2D(2.0, 3.0),
        initial_eta_seconds=18.5,
        current_eta_seconds=18.5,
    )
    state.put_mission(mission(MissionState.EN_ROUTE))
    summary = state.snapshot([], True).active_missions[0]
    assert summary.initial_eta_seconds == 18.5
    assert summary.initial_eta_at == 119.5


def test_current_eta_uses_central_update_without_changing_initial() -> None:
    state = LiveState()
    state.put_event(event())
    now = time.time()
    state.set_mission_target(
        "event-1-aed-robot1",
        now,
        Point2D(2.0, 3.0),
        initial_eta_seconds=18.5,
        current_eta_seconds=18.5,
    )
    state.put_mission(mission(MissionState.EN_ROUTE))
    state.set_mission_eta("event-1-aed-robot1", 23.0, time.time())
    summary = state.snapshot([], True).active_missions[0]
    assert summary.eta_seconds == pytest.approx(23.0, abs=0.1)
    assert summary.initial_eta_seconds == 18.5


def test_sensor_recovery_states_are_merged_into_robot_snapshot() -> None:
    state = LiveState()
    state.put_lidar_state("robot1", "FAULT")
    state.put_fallback_state("robot1", "ACTIVE")
    state.put_robot(RobotSnapshot(
        robot_id="robot1",
        stamp=time.time(),
        position=Point2D(0.0, 0.0),
        yaw_deg=0.0,
        battery_percentage=90.0,
        availability=RobotAvailability.BUSY,
        role=RobotRole.AED_DELIVERY,
        mission_id="event-1-aed-robot1",
        is_docked=False,
        network_ok=True,
        localization_ok=True,
        nav2_ok=True,
        emergency_stop=False,
        path_valid=True,
        estimated_path_cost=1.0,
        last_heartbeat=time.time(),
        detail="",
    ))

    robot = state.snapshot([], True).robots[0]
    assert robot.lidar_state == "FAULT"
    assert robot.lidar_ok is False
    assert robot.fallback_state == "ACTIVE"
    assert robot.healthy is False
