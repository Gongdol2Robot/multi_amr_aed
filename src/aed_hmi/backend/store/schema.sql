-- 관제 이력 저장소.
--
-- 목적은 두 가지다. 사후 검토("그때 왜 늦었나")와 통계("평균 도착 시간").
-- 그래서 상태 전이를 하나도 버리지 않고 모두 남긴다. 요약만 남기면
-- 재할당이 몇 번 있었는지, 어디서 막혔는지 되짚을 수 없다.
--
-- 시각은 전부 UTC epoch 초(REAL)다. 로컬 시간대는 화면에서 붙인다.

PRAGMA journal_mode = WAL;      -- 읽는 중에도 기록이 막히지 않게
PRAGMA foreign_keys = ON;

-- 신고 또는 웹캠 검출로 생긴 응급 이벤트.
CREATE TABLE IF NOT EXISTS emergency_events (
    event_id                TEXT PRIMARY KEY,
    detected_at             REAL NOT NULL,
    map_x                   REAL NOT NULL,
    map_y                   REAL NOT NULL,
    frame_id                TEXT NOT NULL DEFAULT '',
    confidence              REAL NOT NULL DEFAULT 0,
    consecutive_detections  INTEGER NOT NULL DEFAULT 0,
    status                  TEXT NOT NULL,
    source_id               TEXT NOT NULL DEFAULT '',
    camera_id               TEXT NOT NULL DEFAULT '',
    zone_id                 TEXT NOT NULL DEFAULT '',
    -- 신고 접수 시각. 웹캠 검출이면 detected_at 과 같고,
    -- 119 연계면 그쪽에서 준 시각이 들어온다.
    called_at               REAL NOT NULL,
    updated_at              REAL NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_events_called
    ON emergency_events (called_at DESC);

-- 배정 한 건. 재할당하면 assignment_version 이 올라간 새 행이 생긴다.
CREATE TABLE IF NOT EXISTS mission_assignments (
    mission_id          TEXT NOT NULL,
    assignment_version  INTEGER NOT NULL,
    event_id            TEXT NOT NULL,
    robot_id            TEXT NOT NULL,
    role                TEXT NOT NULL,
    target_x            REAL NOT NULL,
    target_y            REAL NOT NULL,
    assigned_at         REAL NOT NULL,
    PRIMARY KEY (mission_id, assignment_version),
    FOREIGN KEY (event_id) REFERENCES emergency_events (event_id)
);

CREATE INDEX IF NOT EXISTS idx_assignments_event
    ON mission_assignments (event_id);

-- 상태 전이 이력. 임무 재구성의 근거이므로 절대 갱신하지 않고 덧붙이기만 한다.
CREATE TABLE IF NOT EXISTS mission_events (
    id                  INTEGER PRIMARY KEY AUTOINCREMENT,
    mission_id          TEXT NOT NULL,
    event_id            TEXT NOT NULL,
    robot_id            TEXT NOT NULL,
    assignment_version  INTEGER NOT NULL,
    state               TEXT NOT NULL,
    stamp               REAL NOT NULL,
    reason              TEXT NOT NULL DEFAULT ''
);

CREATE INDEX IF NOT EXISTS idx_mission_events_mission
    ON mission_events (mission_id, stamp);
CREATE INDEX IF NOT EXISTS idx_mission_events_stamp
    ON mission_events (stamp DESC);

-- 로봇 상태 표본. 전량 저장하면 금방 커지므로 기록 주기를 두고 남긴다.
-- 사후에 "그 시점 배터리가 얼마였나"를 보기 위한 것이다.
CREATE TABLE IF NOT EXISTS robot_samples (
    id                  INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id            TEXT NOT NULL,
    stamp               REAL NOT NULL,
    map_x               REAL NOT NULL,
    map_y               REAL NOT NULL,
    yaw_deg             REAL NOT NULL,
    speed_mps           REAL NOT NULL,
    battery_percentage  REAL NOT NULL,
    availability        TEXT NOT NULL,
    role                TEXT NOT NULL,
    mission_id          TEXT NOT NULL DEFAULT '',
    network_ok          INTEGER NOT NULL,
    localization_ok     INTEGER NOT NULL,
    nav2_ok             INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_robot_samples
    ON robot_samples (robot_id, stamp DESC);
