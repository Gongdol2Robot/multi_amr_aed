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
            # foreign_keys 는 연결마다 켜야 한다. schema.sql 의 PRAGMA 는
            # 그것을 실행한 첫 연결에만 걸린다. 정작 쓰기는 ROS 스레드의
            # 다른 연결에서 일어나므로, 여기서 켜지 않으면 없는 event_id 로
            # 배정이 들어가도 그대로 통과한다.
            # (journal_mode=WAL 은 파일 속성이라 한 번이면 된다.)
            connection.execute("PRAGMA foreign_keys = ON")
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

    def last_event_sequence(self, prefix: str) -> int:
        """`prefix-0007` 같은 id 들 중 가장 큰 번호. 없으면 0.

        시뮬레이터가 매 실행마다 1 번부터 다시 세면, 지난 실행의 임무와
        id 가 겹쳐 이력이 한 줄에 뒤섞인다. 시연 도중 서버를 한 번 껐다
        켜면 바로 드러난다. 이어서 세도록 마지막 번호를 알려준다.
        """
        row = self._connection().execute(
            "SELECT event_id FROM emergency_events WHERE event_id LIKE ?",
            (f"{prefix}-%",),
        ).fetchall()
        best = 0
        for (event_id,) in row:
            tail = event_id.rsplit("-", 1)[-1]
            if tail.isdigit():
                best = max(best, int(tail))
        return best

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

    def upsert_eta_record(self, record) -> None:
        """도착 예상·실제 한 쌍. 같은 건이 다시 오면 덮어쓴다.

        TRANSIENT_LOCAL 토픽이라 관제가 다시 뜨면 지난 결과가 한 번 더
        온다. INSERT 만 하면 그때마다 UNIQUE 위반이 나고, 무시하면 값이
        고쳐졌을 때 반영이 안 된다. 덮어쓰는 편이 양쪽을 다 막는다.
        """
        self._connection().execute(
            """
            INSERT INTO eta_records (
                request_id, robot_id, predicted_sec, actual_sec,
                error_sec, status, stamp
            ) VALUES (?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT (request_id, robot_id) DO UPDATE SET
                predicted_sec = excluded.predicted_sec,
                actual_sec    = excluded.actual_sec,
                error_sec     = excluded.error_sec,
                status        = excluded.status,
                stamp         = excluded.stamp
            """,
            (record.request_id, record.robot_id, record.predicted_sec,
             record.actual_sec, record.error_sec, record.status,
             record.stamp),
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
            # ETA 결과는 응급 요청(event_id)과 로봇별로 한 건씩 저장된다.
            # 복귀도 같은 event_id/robot_id 를 쓰므로 AED 임무에만 연결해야
            # 앞서 끝난 출동 ETA가 복귀 이력에 잘못 표시되지 않는다.
            eta = None
            if "-aed-" in row["mission_id"] and last:
                eta = self._connection().execute(
                    """
                    SELECT predicted_sec, actual_sec, error_sec
                    FROM eta_records
                    WHERE request_id = ? AND robot_id = ?
                    ORDER BY stamp DESC LIMIT 1
                    """,
                    (row["event_id"], last["robot_id"]),
                ).fetchone()
            error_rate = None
            if eta and eta["actual_sec"] > 0.0:
                error_rate = (
                    abs(eta["actual_sec"] - eta["predicted_sec"])
                    / eta["actual_sec"]
                    * 100.0
                )
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
                predicted_eta_seconds=(
                    eta["predicted_sec"] if eta else None
                ),
                actual_travel_seconds=(eta["actual_sec"] if eta else None),
                eta_error_rate_percent=error_rate,
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

    def eta_accuracy_stats(self) -> dict:
        """예상이 얼마나 맞았나. ETA 계수를 고칠 근거다.

        평균 오차(bias)와 절대 오차(정확도)를 따로 낸다. 30초 늦고 30초
        빠른 두 건은 평균이 0 이라 "정확하다"로 보이지만 실제로는 둘 다
        30초씩 틀렸다. 부호가 상쇄되지 않는 값이 함께 있어야 한다.

        늦은 건수도 센다. 관제에서 문제가 되는 것은 예상보다 **늦는** 쪽
        뿐이다. 빨리 도착하는 것은 아무도 항의하지 않는다.
        """
        row = self._connection().execute(
            """
            SELECT
                COUNT(*)                                   AS total,
                AVG(error_sec)                             AS avg_error_sec,
                AVG(ABS(error_sec))                        AS avg_abs_error_sec,
                MAX(ABS(error_sec))                        AS max_abs_error_sec,
                AVG(predicted_sec)                         AS avg_predicted_sec,
                AVG(actual_sec)                            AS avg_actual_sec,
                SUM(CASE WHEN error_sec > 0 THEN 1 ELSE 0 END) AS late_count
            FROM eta_records
            """
        ).fetchone()
        total = row["total"] or 0
        return {
            "total": total,
            "avg_error_sec": row["avg_error_sec"],
            "avg_abs_error_sec": row["avg_abs_error_sec"],
            "max_abs_error_sec": row["max_abs_error_sec"],
            "avg_predicted_sec": row["avg_predicted_sec"],
            "avg_actual_sec": row["avg_actual_sec"],
            "late_count": row["late_count"] or 0,
        }

    def recent_eta_records(self, limit: int = 20) -> list[dict]:
        rows = self._connection().execute(
            """
            SELECT request_id, robot_id, predicted_sec, actual_sec,
                   error_sec, status, stamp
            FROM eta_records ORDER BY stamp DESC LIMIT ?
            """,
            (limit,),
        ).fetchall()
        return [dict(row) for row in rows]

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
