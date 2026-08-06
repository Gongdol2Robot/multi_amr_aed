"""SQLite 접근. SQL 은 전부 여기 있고 밖으로 새지 않는다.

호출자는 도메인 타입만 주고받는다. 그래야 저장 방식이 바뀌어도 화면과
ROS 쪽이 안 흔들린다.

쓰기는 ROS 콜백 스레드에서, 읽기는 FastAPI 요청 스레드에서 온다. 그래서
연결을 스레드마다 따로 만든다(sqlite3 연결은 스레드 간 공유가 안 된다).
"""

import os
import sqlite3
import threading
import time
from typing import Optional

from ..domain.enums import MissionState, mission_state_from_name
from ..domain.models import (
    EmergencyEventSnapshot,
    MissionEvent,
    MissionSummary,
    Point2D,
    RobotSnapshot,
)

SCHEMA_PATH = os.path.join(os.path.dirname(__file__), "schema.sql")


class Repository:
    """이력 저장소. 스레드마다 연결을 따로 연다."""

    def __init__(self, database_path: str) -> None:
        self.database_path = database_path
        self._local = threading.local()
        directory = os.path.dirname(os.path.abspath(database_path))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(SCHEMA_PATH, encoding="utf-8") as handle:
            self._connection().executescript(handle.read())

    def _connection(self) -> sqlite3.Connection:
        connection = getattr(self._local, "connection", None)
        if connection is None:
            connection = sqlite3.connect(
                self.database_path, timeout=5.0, isolation_level=None
            )
            connection.row_factory = sqlite3.Row
            self._local.connection = connection
        return connection

    def close(self) -> None:
        connection = getattr(self._local, "connection", None)
        if connection is not None:
            connection.close()
            self._local.connection = None

    # ------------------------------------------------------------------
    # 쓰기
    # ------------------------------------------------------------------

    def upsert_event(
        self, event: EmergencyEventSnapshot, called_at: Optional[float] = None
    ) -> None:
        """같은 event_id 가 다시 오면 상태만 갱신한다.

        called_at 은 최초 1회만 기록한다. 상태가 바뀔 때마다 덮어쓰면
        신고 시각이 계속 밀려 응답 시간 통계가 무의미해진다.
        """
        now = time.time()
        self._connection().execute(
            """
            INSERT INTO emergency_events (
                event_id, detected_at, map_x, map_y, frame_id, confidence,
                consecutive_detections, status, source_id, camera_id, zone_id,
                called_at, updated_at
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT (event_id) DO UPDATE SET
                status = excluded.status,
                confidence = excluded.confidence,
                consecutive_detections = excluded.consecutive_detections,
                map_x = excluded.map_x,
                map_y = excluded.map_y,
                updated_at = excluded.updated_at
            """,
            (
                event.event_id, event.detected_at,
                event.location.x, event.location.y, event.frame_id,
                event.confidence, event.consecutive_detections,
                event.status.value, event.source_id, event.camera_id,
                event.zone_id,
                called_at if called_at is not None else event.detected_at,
                now,
            ),
        )

    def insert_assignment(
        self, mission_id: str, version: int, event_id: str, robot_id: str,
        role: str, target: Point2D, assigned_at: float,
    ) -> None:
        self._connection().execute(
            """
            INSERT OR IGNORE INTO mission_assignments (
                mission_id, assignment_version, event_id, robot_id, role,
                target_x, target_y, assigned_at
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (mission_id, version, event_id, robot_id, role,
             target.x, target.y, assigned_at),
        )

    def insert_mission_event(self, event: MissionEvent) -> None:
        self._connection().execute(
            """
            INSERT INTO mission_events (
                mission_id, event_id, robot_id, assignment_version,
                state, stamp, reason
            ) VALUES (?, ?, ?, ?, ?, ?, ?)
            """,
            (event.mission_id, event.event_id, event.robot_id,
             event.assignment_version, event.state.value, event.stamp,
             event.reason),
        )

    def insert_robot_sample(self, robot: RobotSnapshot) -> None:
        self._connection().execute(
            """
            INSERT INTO robot_samples (
                robot_id, stamp, map_x, map_y, yaw_deg, speed_mps,
                battery_percentage, availability, role, mission_id,
                network_ok, localization_ok, nav2_ok
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (robot.robot_id, robot.stamp, robot.position.x, robot.position.y,
             robot.yaw_deg, robot.speed_mps, robot.battery_percentage,
             robot.availability.value, robot.role.value, robot.mission_id,
             int(robot.network_ok), int(robot.localization_ok),
             int(robot.nav2_ok)),
        )

    # ------------------------------------------------------------------
    # 읽기
    # ------------------------------------------------------------------

    def recent_missions(self, limit: int = 50) -> list[MissionSummary]:
        """임무별 요약. 상태 전이 이력에서 되짚어 만든다.

        요약을 따로 저장해 두지 않는 이유는, 이력과 요약이 어긋나는 순간
        어느 쪽을 믿어야 할지 알 수 없게 되기 때문이다. 이력만 진실로 둔다.
        """
        rows = self._connection().execute(
            """
            SELECT
                m.mission_id,
                MAX(m.event_id)            AS event_id,
                MAX(m.assignment_version)  AS assignment_version,
                COUNT(DISTINCT m.assignment_version) - 1 AS reassignments,
                MIN(e.called_at)           AS called_at,
                MIN(CASE WHEN m.state = 'dispatching' THEN m.stamp END)
                                           AS dispatched_at,
                MIN(CASE WHEN m.state IN ('arrived', 'completed')
                         THEN m.stamp END) AS arrived_at,
                MAX(m.stamp)               AS last_stamp
            FROM mission_events AS m
            LEFT JOIN emergency_events AS e ON e.event_id = m.event_id
            GROUP BY m.mission_id
            ORDER BY last_stamp DESC
            LIMIT ?
            """,
            (limit,),
        ).fetchall()

        summaries: list[MissionSummary] = []
        for row in rows:
            last = self._connection().execute(
                """
                SELECT robot_id, state FROM mission_events
                WHERE mission_id = ? ORDER BY stamp DESC, id DESC LIMIT 1
                """,
                (row["mission_id"],),
            ).fetchone()
            target = self._connection().execute(
                """
                SELECT target_x, target_y FROM mission_assignments
                WHERE mission_id = ?
                ORDER BY assignment_version DESC LIMIT 1
                """,
                (row["mission_id"],),
            ).fetchone()
            reasons = [
                item["reason"]
                for item in self._connection().execute(
                    """
                    SELECT DISTINCT reason FROM mission_events
                    WHERE mission_id = ? AND reason <> ''
                    ORDER BY stamp
                    """,
                    (row["mission_id"],),
                ).fetchall()
            ]
            summaries.append(MissionSummary(
                mission_id=row["mission_id"],
                event_id=row["event_id"] or "",
                robot_id=last["robot_id"] if last else "",
                target=Point2D(
                    target["target_x"] if target else 0.0,
                    target["target_y"] if target else 0.0,
                ),
                called_at=row["called_at"] or row["last_stamp"],
                dispatched_at=row["dispatched_at"],
                arrived_at=row["arrived_at"],
                final_state=(
                    mission_state_from_name(last["state"])
                    if last else MissionState.ASSIGNED
                ),
                assignment_version=row["assignment_version"] or 0,
                reassignment_count=max(row["reassignments"] or 0, 0),
                failure_reasons=reasons,
            ))
        return summaries

    def mission_timeline(self, mission_id: str) -> list[MissionEvent]:
        rows = self._connection().execute(
            """
            SELECT mission_id, event_id, robot_id, assignment_version,
                   state, stamp, reason
            FROM mission_events WHERE mission_id = ?
            ORDER BY stamp, id
            """,
            (mission_id,),
        ).fetchall()
        return [
            MissionEvent(
                mission_id=row["mission_id"],
                event_id=row["event_id"],
                robot_id=row["robot_id"],
                assignment_version=row["assignment_version"],
                state=mission_state_from_name(row["state"]),
                stamp=row["stamp"],
                reason=row["reason"],
            )
            for row in rows
        ]

    def response_time_stats(self) -> dict:
        """도착까지 걸린 시간 통계. 관제실에서 가장 자주 보는 숫자다."""
        row = self._connection().execute(
            """
            WITH per_mission AS (
                SELECT
                    m.mission_id,
                    MIN(e.called_at) AS called_at,
                    MIN(CASE WHEN m.state IN ('arrived', 'completed')
                             THEN m.stamp END) AS arrived_at
                FROM mission_events AS m
                LEFT JOIN emergency_events AS e ON e.event_id = m.event_id
                GROUP BY m.mission_id
            )
            SELECT
                COUNT(*)                                        AS total,
                SUM(CASE WHEN arrived_at IS NOT NULL THEN 1 ELSE 0 END)
                                                                AS arrived,
                AVG(CASE WHEN arrived_at IS NOT NULL
                         THEN arrived_at - called_at END)       AS avg_seconds,
                MIN(CASE WHEN arrived_at IS NOT NULL
                         THEN arrived_at - called_at END)       AS min_seconds,
                MAX(CASE WHEN arrived_at IS NOT NULL
                         THEN arrived_at - called_at END)       AS max_seconds
            FROM per_mission
            """
        ).fetchone()
        return {
            "total": row["total"] or 0,
            "arrived": row["arrived"] or 0,
            "avg_seconds": row["avg_seconds"],
            "min_seconds": row["min_seconds"],
            "max_seconds": row["max_seconds"],
        }

    def travel_time_stats(self) -> dict:
        """출동 지시부터 도착까지 걸린 시간. ETA 계수를 조정하는 근거다.

        response_time_stats 와 다르다. 저쪽은 신고부터 도착까지라 배정에
        걸린 시간이 섞여 있다. ETA 는 "지금부터 도착까지"를 예측하므로
        이동 구간만 떼어내야 비교가 된다.
        """
        row = self._connection().execute(
            """
            WITH per_mission AS (
                SELECT
                    mission_id,
                    MIN(CASE WHEN state = 'dispatching' THEN stamp END)
                        AS started_at,
                    MIN(CASE WHEN state IN ('arrived', 'completed')
                             THEN stamp END) AS arrived_at
                FROM mission_events
                GROUP BY mission_id
            )
            SELECT
                COUNT(*) AS total,
                AVG(arrived_at - started_at) AS avg_seconds,
                MIN(arrived_at - started_at) AS min_seconds,
                MAX(arrived_at - started_at) AS max_seconds
            FROM per_mission
            WHERE started_at IS NOT NULL AND arrived_at IS NOT NULL
              AND arrived_at > started_at
            """
        ).fetchone()
        return {
            "total": row["total"] or 0,
            "avg_seconds": row["avg_seconds"],
            "min_seconds": row["min_seconds"],
            "max_seconds": row["max_seconds"],
        }

    def robot_track(self, robot_id: str, limit: int = 300) -> list[dict]:
        """최근 이동 궤적. 지도 위에 선으로 그린다."""
        rows = self._connection().execute(
            """
            SELECT stamp, map_x, map_y, speed_mps, battery_percentage
            FROM robot_samples WHERE robot_id = ?
            ORDER BY stamp DESC LIMIT ?
            """,
            (robot_id, limit),
        ).fetchall()
        return [dict(row) for row in reversed(rows)]
