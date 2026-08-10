from backend.domain.enums import EventStatus, MissionState
from backend.domain.models import (
    EmergencyEventSnapshot,
    EtaRecord,
    MissionEvent,
    Point2D,
)
from backend.store.repository import Repository


def test_recent_mission_contains_eta_accuracy_for_aed_only(tmp_path) -> None:
    repository = Repository(str(tmp_path / "hmi.sqlite3"))
    event_id = "emergency-001"
    mission_id = f"{event_id}-aed-robot2"
    repository.upsert_event(EmergencyEventSnapshot(
        event_id=event_id,
        detected_at=100.0,
        location=Point2D(1.0, 2.0),
        frame_id="map",
        confidence=1.0,
        consecutive_detections=1,
        status=EventStatus.CONFIRMED,
        source_id="operator",
        camera_id="",
        zone_id="",
        crowd_level=2,
    ))
    stored = repository._connection().execute(
        "SELECT crowd_level FROM emergency_events WHERE event_id = ?",
        (event_id,),
    ).fetchone()
    assert stored["crowd_level"] == 2
    repository.insert_assignment(
        mission_id, 1, event_id, "robot2", "aed_delivery",
        Point2D(2.0, 3.0), 101.0,
    )
    for state, stamp in (
        (MissionState.DISPATCHING, 101.0),
        (MissionState.ARRIVED, 121.0),
    ):
        repository.insert_mission_event(MissionEvent(
            mission_id=mission_id,
            event_id=event_id,
            robot_id="robot2",
            assignment_version=1,
            state=state,
            stamp=stamp,
            reason="",
        ))
    repository.upsert_eta_record(EtaRecord(
        request_id=event_id,
        robot_id="robot2",
        predicted_sec=18.0,
        actual_sec=20.0,
        status="ARRIVED",
        stamp=121.0,
    ))

    summary = repository.recent_missions()[0]
    assert summary.predicted_eta_seconds == 18.0
    assert summary.actual_travel_seconds == 20.0
    assert summary.eta_error_rate_percent == 10.0

    return_id = f"{event_id}-return-robot2"
    repository.insert_mission_event(MissionEvent(
        mission_id=return_id,
        event_id=event_id,
        robot_id="robot2",
        assignment_version=2,
        state=MissionState.DISPATCHING,
        stamp=122.0,
        reason="",
    ))
    returned = next(
        item for item in repository.recent_missions()
        if item.mission_id == return_id
    )
    assert returned.predicted_eta_seconds is None
    assert returned.eta_error_rate_percent is None

    repository.close()
