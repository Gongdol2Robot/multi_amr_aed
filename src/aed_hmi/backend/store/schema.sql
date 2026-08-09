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
    crowd_level             INTEGER NOT NULL DEFAULT 255,
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

-- 도착 예상과 실제를 한 쌍으로 잰 결과. 출동 한 건이 끝날 때 한 줄 생긴다.
--
-- mission_events 와 따로 두는 이유: 저쪽은 "무슨 일이 있었나"를 남기는
-- 이력이고 이건 "예상이 얼마나 맞았나"를 재는 측정값이다. 성격이 달라서
-- 섞으면 둘 다 읽기 어려워진다.
--
-- 외래키를 걸지 않는다. 이 값은 multi_robot_emergency 가 자기 request_id
-- 로 보내는데, 그쪽 신고가 관제 DB 를 거쳐 들어왔다는 보장이 없다.
-- 측정값을 신고 기록에 종속시키면 신고를 못 받은 구간의 측정까지 잃는다.
CREATE TABLE IF NOT EXISTS eta_records (
    request_id          TEXT NOT NULL,
    robot_id            TEXT NOT NULL,
    predicted_sec       REAL NOT NULL,
    actual_sec          REAL NOT NULL,
    error_sec           REAL NOT NULL,
    status              TEXT NOT NULL DEFAULT '',
    stamp               REAL NOT NULL,
    -- 재할당되면 로봇마다 한 줄씩 남는다. 같은 로봇이 같은 요청을 두 번
    -- 도착할 일은 없으므로 이 둘이면 한 건이 특정된다.
    PRIMARY KEY (request_id, robot_id)
);

CREATE INDEX IF NOT EXISTS idx_eta_records_stamp
    ON eta_records (stamp DESC);
