# DB 설계와 쿼리

표 네 개의 키·인덱스를 왜 그렇게 잡았고, 화면이 부르는 쿼리를 어떻게
짰는지 적는다. 어떤 값이 어느 인터페이스에서 오는지는
[db_interfaces.md](db_interfaces.md) 에 있다.

스키마는 `src/aed_hmi/backend/store/schema.sql`,
쿼리는 `src/aed_hmi/backend/store/repository.py` 한 곳에만 있다.
**SQL 은 이 파일 밖으로 나가지 않는다.** API 나 화면이 직접 질의하기
시작하면 스키마를 바꿀 때 어디까지 영향이 가는지 알 수 없다.

---

## 1. 설계의 축 — 요약을 저장하지 않는다

이 DB 의 성격을 정하는 결정이 하나 있다.

> **임무 요약(언제 출발해 언제 도착했나)을 저장하지 않는다.
> 상태 전이만 남기고, 요약은 물어볼 때마다 되짚어 만든다.**

요약 표를 따로 두면 이력과 어긋나는 순간이 온다. 상태는 `arrived` 인데
요약의 `arrived_at` 이 비어 있으면 어느 쪽을 믿어야 하는지 알 수 없고,
"그때 왜 늦었나"를 되짚을 근거가 사라진다.

그래서 `mission_events` 는 **덧붙이기만 하고 절대 갱신하지 않는다.**
`UPDATE` 도 `DELETE` 도 없다. 재할당이 몇 번이었는지, 어디서 막혔는지는
전부 이 표에서 다시 계산된다.

값은 되짚기 0.19 ms 다(임무 70건, 상태 전이 413행 기준). 요약을 미리
저장해 얻을 이득보다, 두 벌이 어긋날 위험이 크다.

---

## 2. 기본키

| 표 | 기본키 | 왜 |
|---|---|---|
| `emergency_events` | `event_id` (TEXT) | 신고 한 건에 하나. 발행 노드가 정한 자연키다 |
| `mission_assignments` | `(mission_id, assignment_version)` **복합키** | 재할당이 같은 임무의 다른 판이다 |
| `mission_events` | `id` (INTEGER AUTOINCREMENT) **대리키** | 같은 임무·같은 상태가 여러 번 올 수 있다 |
| `robot_samples` | `id` (INTEGER AUTOINCREMENT) **대리키** | 표본에는 자연키가 없다 |

### 왜 `event_id` 는 자연키인가

`event_id` 는 검출한 노드가 만든다(`camera_open-aa9bc4e8`). DB 가 번호를
새로 매기면 로그의 id 와 DB 의 id 가 달라져, 현장 로그와 관제 기록을
맞춰 볼 수 없다. 같은 이벤트가 상태만 바꿔 여러 번 오는 것도
`INSERT ... ON CONFLICT(event_id) DO UPDATE` 로 자연스럽게 처리된다.

### 왜 배정은 복합키인가

재할당은 **새 임무가 아니라 같은 임무의 다음 판**이다.

```
mission_id=evt-0042-aed, assignment_version=1, robot_id=robot1   ← 막힘
mission_id=evt-0042-aed, assignment_version=2, robot_id=robot2   ← 대신 감
```

`mission_id` 만 키로 두면 재할당 때 앞의 행을 덮어써야 하고, 그러면
"robot1 이 먼저 갔다가 실패했다"는 사실이 사라진다. `(mission_id,
assignment_version)` 이면 두 판이 모두 남고, `INSERT OR IGNORE` 로
같은 배정이 두 번 들어와도 조용히 넘어간다.

### 왜 상태 전이는 대리키인가

`(mission_id, state)` 를 키로 삼고 싶어지지만 안 된다. 재할당하면
`en_route` 가 두 번 나오고(1판 robot1, 2판 robot2), 복구하면
`recovery_wait → recovery_resumed → en_route` 로 같은 상태가 또 나온다.
`stamp` 를 넣어도 같은 밀리초에 두 건이 들어오면 충돌한다.

이력은 **일어난 순서 그대로 쌓는 것**이 목적이므로 대리키가 맞다.
`id` 는 정렬의 2차 기준으로도 쓴다(`ORDER BY stamp DESC, id DESC`).
같은 `stamp` 를 가진 두 전이의 순서를 가르는 것은 삽입 순서뿐이다.

---

## 3. 외래키

```sql
mission_assignments.event_id  →  emergency_events.event_id
```

**외래키는 이 하나뿐이다.** 배정은 반드시 신고가 있어야 생긴다. 신고
없는 배정이 들어오면 그 자체가 버그이므로 DB 가 막는 편이 낫다.

`mission_events` 에는 외래키를 걸지 않았다. 이력은 **들어온 그대로 남기는
것이 목적**이라, 신고 기록이 유실됐다고 해서 그 뒤의 상태 전이까지 버리면
"무슨 일이 있었는지"를 더 모르게 된다. 대신 `recent_missions` 가
`LEFT JOIN` 으로 읽어, 신고 기록이 없어도 임무는 보이고 `called_at` 만
비게 한다.

`robot_samples` 의 `mission_id` 에도 걸지 않았다. 임무 없이 돌아다니는
동안에도 표본은 남아야 해서 빈 문자열이 들어온다.

### 연결마다 켜야 한다

`PRAGMA foreign_keys` 는 **연결별 설정**이다. `schema.sql` 안에 써 두면
그 스크립트를 실행한 첫 연결에만 걸린다. 이 저장소는 스레드마다 연결을
따로 여는데(ROS 스레드가 쓰고 FastAPI 스레드가 읽는다), 정작 쓰기가
일어나는 연결에서 꺼져 있으면 아무것도 막지 못한다.

그래서 `_connection()` 이 새 연결을 열 때마다 켠다.

```python
connection.execute("PRAGMA foreign_keys = ON")
```

`journal_mode = WAL` 은 파일 속성이라 한 번이면 된다. 둘을 같이 두면
안 되는 이유가 여기 있다.

---

## 4. 인덱스

| 인덱스 | 대상 | 무엇을 위해 |
|---|---|---|
| `idx_events_called` | `emergency_events (called_at DESC)` | 최근 신고부터 |
| `idx_assignments_event` | `mission_assignments (event_id)` | 신고 하나의 배정들 |
| `idx_mission_events_mission` | `mission_events (mission_id, stamp)` | 임무 하나의 이력을 시각순으로 |
| `idx_mission_events_stamp` | `mission_events (stamp DESC)` | 최근 전이부터 |
| `idx_robot_samples` | `robot_samples (robot_id, stamp DESC)` | 로봇 하나의 최근 궤적 |

복합 인덱스의 **열 순서**가 중요하다. `(mission_id, stamp)` 는
"임무를 고른 뒤 시각순"에 쓰이고, `(stamp, mission_id)` 였다면 그 질의에
못 쓴다. 궤적도 마찬가지로 `(robot_id, stamp DESC)` 다 — 로봇을 먼저
고른다.

요약 쿼리의 실제 실행 계획은 이렇다.

```
SCAN m USING INDEX idx_mission_events_mission
SEARCH e USING COVERING INDEX sqlite_autoindex_emergency_events_1 (event_id=?)
USE TEMP B-TREE FOR ORDER BY
```

`GROUP BY mission_id` 가 인덱스 순서를 그대로 타서 정렬 없이 묶인다.
조인은 `emergency_events` 의 기본키 인덱스로 바로 찾는다. 마지막
`ORDER BY last_stamp DESC` 만 임시 B-tree 를 쓰는데, 이건 집계 **결과**를
정렬하는 것이라 인덱스로 없앨 수 없다.

---

## 5. 쿼리별 설명

### `recent_missions()` — 이력 표

상태 전이에서 임무 하나를 되짚는 핵심 쿼리다.

```sql
SELECT
    m.mission_id,
    MAX(m.assignment_version)                AS assignment_version,
    COUNT(DISTINCT m.assignment_version) - 1 AS reassignments,
    MIN(e.called_at)                         AS called_at,
    MIN(CASE WHEN m.state = 'dispatching' THEN m.stamp END)       AS dispatched_at,
    MIN(CASE WHEN m.state IN ('arrived','completed') THEN m.stamp END) AS arrived_at,
    MAX(m.stamp)                             AS last_stamp
FROM mission_events AS m
LEFT JOIN emergency_events AS e ON e.event_id = m.event_id
GROUP BY m.mission_id
ORDER BY last_stamp DESC
LIMIT ?
```

읽는 요령 몇 가지.

- **`MIN(CASE WHEN ...)`** — 조건부 집계다. 같은 임무의 여러 행 중
  그 상태인 행의 시각만 골라 가장 이른 것을 취한다. 상태별로 표를
  나누지 않고 한 번에 끝낸다.
- **`MIN` 이지 `MAX` 가 아닌 이유** — 복구로 `dispatching` 이 두 번
  나오면 **처음** 출발한 시각이 출동 시각이다. 도착도 `arrived` 뒤에
  `completed` 가 이어지므로 처음 것을 쓴다.
- **`COUNT(DISTINCT assignment_version) - 1`** — 판이 둘이면 재할당 1회다.
- **`LEFT JOIN`** — 신고 기록이 없어도 임무는 보여야 한다. `INNER JOIN`
  이면 그 임무가 표에서 통째로 사라진다.
- **`ORDER BY last_stamp DESC`** — 접수 시각이 아니라 **마지막 움직임**
  순이다. 오래된 신고라도 방금 상태가 바뀌었으면 위로 올라와야 한다.

로봇·목표·실패 사유는 이 쿼리로 못 얻는다. 임무마다 세 번 더 묻는다.

| 무엇 | 어디서 | 왜 따로 |
|---|---|---|
| 마지막 로봇·상태 | `mission_events` `ORDER BY stamp DESC, id DESC LIMIT 1` | 집계로는 "마지막 행의 다른 열"을 못 가져온다 |
| 목표 좌표 | `mission_assignments` `ORDER BY assignment_version DESC LIMIT 1` | 재할당됐으면 마지막 판의 목표 |
| 실패 사유 | `mission_events` `WHERE reason <> ''` | 여러 건일 수 있다 |

**이건 N+1 이다.** 30건을 보여주면 `1 + 30×3 = 91` 번 질의한다. 지금은
전부 합쳐 1 ms 안쪽이고 화면이 5초마다 한 번 부르니 문제가 아니다.
행이 수십만으로 늘면 윈도 함수(`ROW_NUMBER() OVER (PARTITION BY
mission_id ORDER BY stamp DESC)`)로 한 방에 합치면 된다. **지금 그렇게
하지 않은 이유는, 세 갈래로 나뉜 지금 쿼리가 읽기 쉽기 때문이다.**
느려진 뒤에 바꿔도 늦지 않다.

### `response_time_stats()` / `travel_time_stats()` — 통계

둘 다 CTE 로 임무별 시각을 먼저 만들고 그 위에서 집계한다.

```sql
WITH per_mission AS (
    SELECT mission_id,
           MIN(e.called_at) AS called_at,
           MIN(CASE WHEN state IN ('arrived','completed') THEN stamp END) AS arrived_at
    FROM mission_events ... GROUP BY mission_id
)
SELECT COUNT(*), AVG(arrived_at - called_at), MIN(...), MAX(...) FROM per_mission
```

두 쿼리를 나눈 이유가 이 문서에서 제일 중요한 대목이다.

| | 재는 구간 | 쓰는 곳 |
|---|---|---|
| `response_time_stats` | **접수 → 도착** | 관제실이 보는 숫자 |
| `travel_time_stats` | **출동 → 도착** | ETA 계수를 맞추는 근거 |

응답 시간에는 **배정에 걸린 시간이 섞여 있다.** ETA 는 "지금부터
도착까지"를 예측하므로 이동 구간만 떼어내야 예측과 실측을 비교할 수
있다. 하나로 합치면 예측이 틀렸을 때 원인이 배정인지 주행인지 못 가른다.

`travel_time_stats` 에만 `arrived_at > started_at` 조건이 있다. 시각이
어긋난 기록(로봇 시계가 튀거나 전이가 뒤집혀 들어온 경우)이 평균을
음수로 끌고 가는 것을 막는다.

### `robot_track()` — 궤적

```sql
SELECT stamp, map_x, map_y, speed_mps, battery_percentage
FROM robot_samples WHERE robot_id = ?
ORDER BY stamp DESC LIMIT ?
```

`(robot_id, stamp DESC)` 인덱스를 그대로 탄다. **최신부터** 잘라 오고
그리기 직전에 뒤집는다. 오래된 것부터 읽으면 최근 300개를 얻으려고
전체를 스캔해야 한다.

---

## 6. 쓰기 쪽 규칙

| 표 | 구문 | 왜 |
|---|---|---|
| `emergency_events` | `INSERT ... ON CONFLICT(event_id) DO UPDATE` | 같은 신고가 상태만 바꿔 여러 번 온다 |
| `mission_assignments` | `INSERT OR IGNORE` | 같은 배정이 두 번 와도 조용히 넘어간다 |
| `mission_events` | `INSERT` | 이력은 무조건 덧붙인다 |
| `robot_samples` | `INSERT` | 1초에 한 번만 |

`upsert_event` 의 `DO UPDATE` 에서 **`called_at` 은 건드리지 않는다.**

```sql
ON CONFLICT (event_id) DO UPDATE SET
    status = excluded.status,
    confidence = excluded.confidence,
    consecutive_detections = excluded.consecutive_detections,
    map_x = excluded.map_x,
    map_y = excluded.map_y,
    updated_at = excluded.updated_at
```

상태가 바뀔 때마다 신고 시각이 밀리면 응답 시간이 계속 줄어들어
통계가 무의미해진다. **접수 시각은 최초 1회만 쓴다.**

`robot_samples` 는 10Hz 로 오는 것을 **1초에 한 번만** 남긴다
(`Settings.robot_sample_interval_s`). 그대로 넣으면 로봇 2대에 하루
170만 행이다. 솎는 판단은 SQL 이 아니라 `context.py: _maybe_sample()`
에서 한다. DB 에 넣고 나서 지우는 것보다 안 넣는 편이 싸다.

---

## 7. 동시성

ROS 스레드가 쓰고 FastAPI 스레드가 읽는다. 둘이 막지 않게 두 가지를 쓴다.

**WAL** (`PRAGMA journal_mode = WAL`) — 읽는 동안에도 쓰기가 진행된다.
기본 모드였다면 통계 쿼리가 도는 사이 로봇 상태 기록이 막힌다.

**스레드마다 연결** — `sqlite3.Connection` 은 스레드 사이에 공유하면
안 된다. `threading.local()` 로 각자 열고, 각 연결에서 `foreign_keys` 를
켠다.

`isolation_level=None` 으로 열어 파이썬의 자동 트랜잭션을 끈다. 쓰기가
전부 단문이라 묶을 것이 없고, 암묵적 트랜잭션이 열린 채로 남아 WAL 이
계속 커지는 것을 막는다.

`timeout=5.0` 은 쓰기 잠금을 기다리는 시간이다. 넘으면 예외가 나는데,
`context.py` 가 잡아서 로그만 남기고 넘어간다. **기록에 실패해도 관제
화면은 계속 떠 있어야 한다.**

---

## 8. 아직 안 한 것

| | 지금 | 언제 해야 하나 |
|---|---|---|
| 오래된 표본 삭제 | 안 지운다 | `robot_samples` 가 며칠치 쌓이면. 시연 기간에는 문제없다 |
| `recent_missions` N+1 | 임무당 3번 더 | 이력이 수만 건이 되면 윈도 함수로 |
| `crowd_level` 열 | 없다 | 메시지에는 있다. "왜 2대가 갔나"를 되짚으려면 필요 |
| 마이그레이션 | `CREATE TABLE IF NOT EXISTS` 뿐 | 열을 추가·변경하게 되면. 지금은 지우고 다시 만들면 된다 |
| 인덱스 재검토 | 행 수가 적어 의미 없음 | 실제 운영 데이터가 쌓인 뒤 `EXPLAIN QUERY PLAN` 으로 |
