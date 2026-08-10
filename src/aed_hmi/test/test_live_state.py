import time

import pytest

from backend.domain.enums import EventStatus, MissionState
from backend.domain.models import (
    CrowdZoneSnapshot,
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


def robot(*, docked: bool) -> RobotSnapshot:
    return RobotSnapshot(
        robot_id="robot1",
        stamp=time.time(),
        position=Point2D(0.0, 0.0),
        yaw_deg=0.0,
        battery_percentage=90.0,
        availability=RobotAvailability.BUSY,
        role=RobotRole.AED_DELIVERY,
        mission_id="event-1-aed-robot1",
        is_docked=docked,
        network_ok=True,
        localization_ok=True,
        nav2_ok=True,
        emergency_stop=False,
        path_valid=True,
        estimated_path_cost=1.0,
        last_heartbeat=time.time(),
        detail="",
    )


def test_crowd_zone_is_included_in_live_snapshot() -> None:
    state = LiveState()
    zone = CrowdZoneSnapshot(
        zone_id="alley_zone",
        polygon=[
            Point2D(-3.0, 1.6), Point2D(-2.9, 0.8),
            Point2D(-1.0, 1.2), Point2D(-1.2, 2.0),
        ],
        level=2,
        level_name="CROWDED",
        person_count=5,
        fresh=True,
        age_sec=0.1,
    )

    state.put_crowd_zone(zone)

    assert state.snapshot([], True).crowd_zone == zone


def test_camera_cancel_does_not_hide_an_active_dispatch() -> None:
    state = LiveState()
    confirmed = event()
    state.put_event(confirmed)
    state.put_mission(mission(MissionState.EN_ROUTE))
    state.put_event(EmergencyEventSnapshot(
        **{**confirmed.__dict__, "status": EventStatus.CANCELED}
    ))
    assert active_event(state) == confirmed


def test_unrelated_detection_does_not_replace_active_mission_banner() -> None:
    state = LiveState()
    confirmed = event()
    state.put_event(confirmed)
    state.put_mission(mission(MissionState.EN_ROUTE))
    state.put_event(event("ignored-event"))
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


def test_dispatch_time_starts_only_after_nav2_and_dock_departure() -> None:
    state = LiveState()
    state.put_event(event())
    state.put_mission(mission(MissionState.DISPATCHING))
    assert state.snapshot([], True).active_missions[0].dispatched_at is None

    accepted = MissionEvent(
        **{
            **mission(MissionState.EN_ROUTE).__dict__,
            "stamp": 105.0,
        }
    )
    state.put_mission(accepted)
    assert state.snapshot([], True).active_missions[0].dispatched_at is None

    state.put_robot(robot(docked=True))
    assert state.snapshot([], True).active_missions[0].dispatched_at is None

    before_departure = time.time()
    state.put_robot(robot(docked=False))
    dispatched_at = state.snapshot([], True).active_missions[0].dispatched_at
    assert dispatched_at is not None
    assert dispatched_at >= before_departure

    later = MissionEvent(**{**accepted.__dict__, "stamp": 110.0})
    state.put_mission(later)
    assert state.snapshot([], True).active_missions[0].dispatched_at == (
        dispatched_at
    )


def test_navigation_error_stays_visible_until_event_is_canceled() -> None:
    state = LiveState()
    confirmed = event()
    state.put_event(confirmed)
    state.set_mission_target(
        "event-1-aed-robot1", 101.0, Point2D(2.0, 3.0),
        role=RobotRole.AED_DELIVERY,
    )
    state.put_mission(mission(MissionState.NAVIGATION_ERROR))
    snapshot = state.snapshot([], True)
    assert snapshot.active_event == confirmed
    assert snapshot.active_missions[0].final_state == (
        MissionState.NAVIGATION_ERROR
    )

    state.put_event(EmergencyEventSnapshot(
        **{**confirmed.__dict__, "status": EventStatus.CANCELED}
    ))
    assert active_event(state) is None


def test_return_mission_does_not_reopen_finished_emergency_banner() -> None:
    state = LiveState()
    state.put_event(event())
    state.put_mission(mission(MissionState.ARRIVED))
    assert active_event(state) is None

    return_id = "event-1-helper-return-robot1"
    state.set_mission_target(
        return_id, 102.0, Point2D(0.0, 0.0), role=RobotRole.RETURN
    )
    state.put_mission(MissionEvent(
        mission_id=return_id,
        event_id="event-1",
        robot_id="robot1",
        assignment_version=2,
        state=MissionState.DISPATCHING,
        stamp=102.0,
        reason="",
    ))
    assert active_event(state) is None


def test_reassignment_supersedes_failed_mission_and_can_finish_event() -> None:
    state = LiveState()
    state.put_event(event())
    state.set_mission_target(
        "event-1-aed-robot1", 101.0, Point2D(2.0, 3.0),
        role=RobotRole.AED_DELIVERY,
    )
    state.put_mission(mission(MissionState.BLOCKED))

    replacement = MissionEvent(
        mission_id="event-1-aed-robot2",
        event_id="event-1",
        robot_id="robot2",
        assignment_version=2,
        state=MissionState.ASSIGNED,
        stamp=102.0,
        reason="executor 응답 대기",
    )
    state.set_mission_target(
        replacement.mission_id, 102.0, Point2D(2.1, 3.1),
        role=RobotRole.AED_DELIVERY,
    )
    state.put_mission(replacement)
    snapshot = state.snapshot([], True)
    assert [item.mission_id for item in snapshot.active_missions] == [
        replacement.mission_id
    ]

    state.put_mission(MissionEvent(
        **{**replacement.__dict__, "state": MissionState.ARRIVED,
           "stamp": 120.0}
    ))
    assert active_event(state) is None


def test_dual_dispatch_banner_closes_when_late_robot_switches_to_return() -> None:
    state = LiveState()
    state.put_event(event())
    first = mission(MissionState.EN_ROUTE)
    second = MissionEvent(
        mission_id="event-1-aed-robot2",
        event_id="event-1",
        robot_id="robot2",
        assignment_version=2,
        state=MissionState.EN_ROUTE,
        stamp=102.0,
        reason="",
    )
    state.set_mission_target(
        first.mission_id, 101.0, Point2D(2.0, 3.0),
        role=RobotRole.AED_DELIVERY,
    )
    state.set_mission_target(
        second.mission_id, 102.0, Point2D(2.0, 3.0),
        role=RobotRole.AED_DELIVERY,
    )
    state.put_mission(first)
    state.put_mission(second)
    state.put_mission(MissionEvent(
        **{**first.__dict__, "state": MissionState.ARRIVED,
           "stamp": 110.0}
    ))
    assert active_event(state) is not None

    return_mission = MissionEvent(
        mission_id="event-1-return-robot2",
        event_id="event-1",
        robot_id="robot2",
        assignment_version=3,
        state=MissionState.ASSIGNED,
        stamp=111.0,
        reason="",
    )
    state.set_mission_target(
        return_mission.mission_id, 111.0, Point2D(0.0, 0.0),
        role=RobotRole.RETURN,
    )
    state.put_mission(return_mission)
    assert active_event(state) is None


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
