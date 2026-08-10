# DB 와 인터페이스 대응표

관제 DB(`src/aed_hmi/backend/store/schema.sql`)에 들어가는 값이 각각 **어느
노드가**, **어떤 통신 방식으로**, **어떤 자료형**으로 보낸 것인지 적는다.

DB 는 ROS 를 모른다. 사이에 `aed_hmi/backend/ros/` 가 있고, 거기서만
`aed_interfaces` 를 import 한다. 그래서 이 문서는 두 겹으로 나뉜다.

```
ROS 인터페이스  →  ros/converters.py  →  domain 모델  →  store/repository.py  →  SQLite
   (uint8 등)        (형 변환)           (파이썬 형)        (SQL)              (TEXT/REAL)
```

인터페이스 자체의 사양과 왜 그 통신 방식인지는 [interfaces.md](interfaces.md)
에 있다. 표의 키·인덱스를 왜 그렇게 잡았고 화면이 부르는 쿼리를 어떻게
짰는지는 [db_queries.md](db_queries.md) 에 있다. 여기서는 **어느 값이
어디서 와서 어느 칸에 들어가는지**만 다룬다.

---

## 1. 통신 방식별 인터페이스 목록

상태 열은 **오늘 돌아가는지**를 적은 것이다. 사양은 확정이지만 구현이 안 된
자리가 있고, 그 자리는 리뷰에서 짚어야 한다.

- **동작** — 노드가 실제로 발행/구독한다
- **뼈대** — 패키지와 노드는 있으나 콜백이 비어 있다(29줄짜리 scaffold)
- **사양만** — 메시지 정의는 확정, 쓰는 노드가 아직 없다

### Topic — 계속 흐르는 상태

| 토픽 | 자료형 | 발행 노드 | 구독 노드 | 상태 | DB 반영 |
|---|---|---|---|---|---|
| `/aed/robot_state` | `aed_interfaces/RobotState` | `robot_state_monitor` | `multi_robot_emergency`, `aed_hmi_bridge` | **뼈대** | `robot_samples` |
| `/aed/mission_status` | `aed_interfaces/MissionStatus` | `multi_robot_emergency`, `mission_executor` | `aed_hmi_bridge`, `event_logger` | 동작 | `mission_events` |
| `/{robot_id}/mission_assignment` | `aed_interfaces/MissionAssignment` | `multi_robot_emergency` | `mission_executor` | 동작 | `mission_assignments` |
| `/aed/emergency_event` | `aed_interfaces/EmergencyEvent` | (없음) | `multi_robot_emergency`, `aed_hmi_bridge` | **발행자 없음** | `emergency_events` |
| `/{camera_id}/vision/emergency_event` | `aed_interfaces/EmergencyEvent` | `vision_detector` | `aed_hmi_bridge` | 동작 | `emergency_events` |
| `/{camera_id}/vision/crowd_level` | `std_msgs/String` → `CrowdLevel` | `vision_detector` | `multi_robot_emergency`, `aed_hmi_bridge` | 동작(형 교체 예정) | 안 남김 |
| `/{camera_id}/vision/detection_summary` | `aed_interfaces/DetectionSummary` | `vision_detector` | `aed_hmi_bridge` | 동작 | 안 남김 |
| `/{camera_id}/vision/debug/compressed` | `sensor_msgs/CompressedImage` | `vision_detector` | `aed_hmi_bridge` | 동작 | 안 남김 |
| `/{camera_id}/vision/person_count` | `std_msgs/UInt32` | `vision_detector` | `aed_hmi_bridge` | 동작 | 안 남김 |
| `/{camera_id}/vision/heartbeat` | `aed_interfaces/Heartbeat` | `vision_detector` | `recovery_manager` | 동작 | 안 남김 |
| `/{robot_id}/sensor_health` | `aed_interfaces/SensorHealth` | `sensor_health_monitor` | `multi_robot_emergency`, `aed_hmi_bridge` | **사양만** | 안 남김 |
| `/{robot_id}/cmd_vel` | `geometry_msgs/Twist` | `sensor_health_monitor` (대체 주행) | Create3 | **사양만** | 안 남김 |
| `/emergency/eta/result` | `std_msgs/String` (JSON) | `multi_robot_emergency` | `aed_hmi_bridge` | 동작 | `eta_records` |
| `/emergency/eta/predicted/{robot_id}` | `std_msgs/Float32` | `multi_robot_emergency` | (안 받음) | 동작 | 안 남김 |
| `/emergency/eta/actual/{robot_id}` | `std_msgs/Float32` | `multi_robot_emergency` | (안 받음) | 동작 | 안 남김 |

`camera_id` 는 `camera_open`, `camera_alley`, `robot1`, `robot2` 다.

관제가 만드는 ROS 노드 이름은 **`aed_hmi_bridge`** 다(`backend/ros/bridge.py`).
`aed_hmi` 패키지의 `hmi_node` 진입점은 쓰지 않는 뼈대다. 화면 서버는
`python3 -m backend.main` 으로 뜨고, 그 안에서 별도 스레드로 이 노드를
만든다. ROS 를 아는 코드가 `backend/ros/` 안에만 있어야 하기 때문이다.

`/aed/emergency_event` 에 **발행자가 없다.** `vision_detector` 는 카메라별
토픽에 내고 `multi_robot_emergency`는 통합 토픽을 듣는데 그 사이를 잇는 노드
(`location_mapper`)가 아직 뼈대뿐이다. 그래서 지금은 검출이 출동으로
이어지지 않는다. 화면은 두 경로를 모두 구독하고 있어 이어붙인 뒤에도
고칠 것이 없다.

### Service — 한 번 묻고 즉시 답

| 서비스 | 자료형 | 서버 | 클라이언트 | 상태 | DB 반영 |
|---|---|---|---|---|---|
| `/aed/report_emergency` | `aed_interfaces/ReportEmergency` | `location_mapper` (예정) | 119/운영자 연계 | **사양만** | `emergency_events` (응답의 `event_id` 로 행이 생김) |

### Action — 오래 걸리고, 진행을 보고, 취소한다

| 액션 | 자료형 | 서버 | 클라이언트 | 상태 | DB 반영 |
|---|---|---|---|---|---|
| `/{robot_id}/deliver_aed` | `aed_interfaces/DeliverAed` | `mission_executor` | `multi_robot_emergency` | **사양만** | `mission_assignments` + `mission_events` |
| `/{robot_id}/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 | `mission_executor` | 동작 | 안 남김 |

`DeliverAed` 는 지금의 `MissionAssignment` topic 을 대체한다. 교체 전까지
`mission_assignments` 표는 topic 으로 채워진다. 표에 들어가는 값 자체는
같아서, 바꿔도 DB 스키마는 손대지 않는다.

---

## 2. 테이블별 대응

### `emergency_events` — 신고 한 건

출처는 둘이고 통신 방식이 다르다. **웹캠 검출은 Topic, 119 연계는 Service**
다. 어느 쪽으로 들어왔는지는 `source_id` 로 갈린다.

| 컬럼 | SQLite 형 | 출처 | 필드 | ROS 형 |
|---|---|---|---|---|
| `event_id` | TEXT PK | EmergencyEvent | `event_id` | `string` |
| `detected_at` | REAL | EmergencyEvent | `detected_at` | `builtin_interfaces/Time` |
| `map_x` / `map_y` | REAL | EmergencyEvent | `location.point.x/.y` | `geometry_msgs/PointStamped` |
| `frame_id` | TEXT | EmergencyEvent | `location.header.frame_id` | `string` |
| `confidence` | REAL | EmergencyEvent | `confidence` | `float32` |
| `consecutive_detections` | INTEGER | EmergencyEvent | `consecutive_detections` | `uint32` |
| `status` | TEXT | EmergencyEvent | `status` | `uint8` → 이름 문자열 |
| `source_id` | TEXT | EmergencyEvent | `source_id` | `string` |
| `camera_id` | TEXT | EmergencyEvent | `camera_id` | `string` |
| `zone_id` | TEXT | EmergencyEvent | `zone_id` | `string` |
| `location_source` | 저장 안 함 | EmergencyEvent | `location_source` | `string` |
| `location_valid` | 저장 안 함 | EmergencyEvent | `location_valid` | `bool` |
| `called_at` | REAL | 최초 1회의 `detected_at` | — | — |
| `updated_at` | REAL | 서버 수신 시각 | — | — |

`status` 는 `DETECTED / CONFIRMED / DISPATCHED / RESOLVED / CANCELED`
(uint8 0~4)를 소문자 이름으로 바꿔 넣는다.

`consecutive_detections`는 하위 호환 필드명이다. 현재 Vision 노드에서는 동일
bbox가 정지 확정 조건을 만족하는 동안 누적된 관측 수를 저장한다.
`location_source`와 `location_valid`는 현재 SQLite에는 저장하지 않고 HMI의
실시간 이벤트 상태에만 보존한다.

같은 `event_id` 가 다시 오면 상태만 갱신하고 `called_at` 은 **덮어쓰지
않는다.** 상태가 바뀔 때마다 신고 시각이 밀리면 응답 시간 통계가 무의미해진다.

### `mission_assignments` — 배정 한 건

**Action 의 Goal** 에 해당한다. 재할당하면 `assignment_version` 이 올라간
새 행이 생기고, 기존 행은 남는다.

| 컬럼 | SQLite 형 | 출처 | 필드 | ROS 형 |
|---|---|---|---|---|
| `mission_id` | TEXT PK¹ | DeliverAed.Goal | `mission_id` | `string` |
| `assignment_version` | INTEGER PK¹ | multi_robot_emergency가 셈 | — | — |
| `event_id` | TEXT FK | DeliverAed.Goal | `event_id` | `string` |
| `robot_id` | TEXT | goal 을 보낸 액션 이름에서 | — | — |
| `role` | TEXT | DeliverAed.Goal | `role` | `uint8` → 이름 문자열 |
| `target_x` / `target_y` | REAL | DeliverAed.Goal | `target.pose.position.x/.y` | `geometry_msgs/PoseStamped` |
| `assigned_at` | REAL | DeliverAed.Goal | `requested_at` | `builtin_interfaces/Time` |

¹ `(mission_id, assignment_version)` 복합 기본키.

### `mission_events` — 상태 전이 이력

**갱신하지 않고 덧붙이기만 한다.** 임무 요약은 저장하지 않고 이 표에서
매번 되짚는다. 요약을 따로 두면 둘이 어긋났을 때 어느 쪽을 믿을지 알 수 없다.

| 컬럼 | SQLite 형 | 출처 | 필드 | ROS 형 |
|---|---|---|---|---|
| `id` | INTEGER PK | 자동 증가 | — | — |
| `mission_id` | TEXT | MissionStatus | `mission_id` | `string` |
| `event_id` | TEXT | MissionStatus | `event_id` | `string` |
| `robot_id` | TEXT | MissionStatus | `robot_id` | `string` |
| `assignment_version` | INTEGER | MissionStatus | `assignment_version` | `uint32` |
| `state` | TEXT | MissionStatus | `status` | `uint8` → 이름 문자열 |
| `stamp` | REAL | MissionStatus | `stamp` | `builtin_interfaces/Time` |
| `reason` | TEXT | MissionStatus | `reason` | `string` |

`state` 는 uint8 0~13 을 이름으로 바꾼 것이다.
`assigned / dispatching / en_route / arrived / completed / canceled /
blocked / network_lost / navigation_error / recovery_wait /
recovery_resumed / helper_requested / helper_en_route / helper_arrived`.

`DeliverAed` 의 Feedback·Result 도 같은 상태값을 쓴다. Action 은 요청한
쪽만 결과를 보므로, 화면과 로그가 모든 임무를 보려면 `MissionStatus`
topic 이 따로 있어야 한다.

### `eta_records` — 도착 예상과 실제

`multi_robot_emergency` 가 출동 한 건이 끝날 때 내는 값이다. **이 토픽만
`std_msgs/String` 에 JSON 이라 자료형이 없다.**

```json
{"actual_arrival_sec":23.5,"error_sec":0.67,"predicted_eta_sec":22.83,
 "request_id":"emergency-002","robot_id":"robot2","stamp_sec":1786011309.024,
 "status":"ARRIVED"}
```

| 컬럼 | SQLite 형 | JSON 칸 |
|---|---|---|
| `request_id` | TEXT PK¹ | `request_id` |
| `robot_id` | TEXT PK¹ | `robot_id` |
| `predicted_sec` | REAL | `predicted_eta_sec` |
| `actual_sec` | REAL | `actual_arrival_sec` |
| `error_sec` | REAL | (**다시 계산한다**) |
| `status` | TEXT | `status` |
| `stamp` | REAL | `stamp_sec` |

¹ `(request_id, robot_id)` 복합 기본키.

`error_sec` 은 JSON 에도 있지만 `actual - predicted` 로 다시 계산한다.
두 값이 어긋나면 어느 쪽이 맞는지 알 수 없고, 세 수를 모두 갖고 있으므로
남의 뺄셈을 믿을 이유가 없다.

`request_id` 는 저쪽의 이벤트 식별자이고, 임무 식별자는 거기에 `-aed` 를
붙인 것이다(`mission_manager.py` 가 그렇게 만든다). 우리 규칙과 같아서
매핑 표가 필요 없다.

**받는 자리에서 한 번 검사한다.** `.msg` 는 칸과 형을 보장하지만 JSON 은
보내는 쪽이 무엇을 넣든 통과한다. `EtaRecord.from_json()` 이 이 시스템에서
형이 보장되지 않은 값이 들어오는 **유일한 통로**이고, 거기서 빠진 칸·숫자가
아닌 값·음수 시간을 걸러 낸다. 깨진 것은 로그를 남기고 버린다. 통계 한
건을 잃는 편이 관제 화면을 끄는 것보다 낫다.

로봇별 `Float32` 두 갈래는 **받지 않는다.** 어느 요청의 값인지가 안 실려
있어서, 두 요청이 겹치면 가릴 수 없다. 요청 id 가 들어 있는 `result`
하나만 받는다.

**QoS 를 맞춰야 한다.** 저쪽은 `TRANSIENT_LOCAL`, 우리 기본 `STATE_QOS`
는 `VOLATILE` 이다. durability 가 다르면 ROS 2 는 연결을 아예 안 맺고
경고도 없다. `topics.LATCHED_QOS` 로 맞췄다. 덕분에 관제가 나중에 떠도
최근 10건을 받는다.

### `robot_samples` — 로봇 상태 표본

10Hz 를 그대로 넣으면 하루에 백만 행이다. **1초에 한 번만** 남긴다
(`Settings.robot_sample_interval_s`).

| 컬럼 | SQLite 형 | 출처 | 필드 | ROS 형 |
|---|---|---|---|---|
| `robot_id` | TEXT | RobotState | `robot_id` | `string` |
| `stamp` | REAL | RobotState | `stamp` | `builtin_interfaces/Time` |
| `map_x` / `map_y` | REAL | RobotState | `pose.pose.position.x/.y` | `geometry_msgs/PoseStamped` |
| `yaw_deg` | REAL | RobotState | `pose.pose.orientation` 에서 계산 | `geometry_msgs/Quaternion` |
| `speed_mps` | REAL | RobotState | `speed_mps` | `float32` |
| `battery_percentage` | REAL | RobotState | `battery_percentage` | `float32` |
| `availability` | TEXT | RobotState | `availability` | `uint8` → 이름 문자열 |
| `role` | TEXT | RobotState | `role` | `uint8` → 이름 문자열 |
| `mission_id` | TEXT | RobotState | `mission_id` | `string` |
| `network_ok` | INTEGER | RobotState | `network_ok` | `bool` → 0/1 |
| `localization_ok` | INTEGER | RobotState | `localization_ok` | `bool` → 0/1 |
| `nav2_ok` | INTEGER | RobotState | `nav2_ok` | `bool` → 0/1 |

DB 에 안 남기고 화면에만 쓰는 필드: `is_docked`, `emergency_stop`,
`path_valid`, `estimated_path_cost`, `last_heartbeat`, `detail`.
`estimated_path_cost` 는 도착 예상 계산에만 쓰고, 매초 남길 값은 아니다.

---

## 3. 시각 — 어느 시점인가

관제에서 따지는 것은 결국 시각이다. **새 통신을 만들지 않고 이미 있는
메시지에서 얻는다.**

| 값 | 어느 시점 | 나오는 곳 | 통신 방식 |
|---|---|---|---|
| 신고 접수 | 최초 검출/신고 | `EmergencyEvent.detected_at` | Topic 또는 Service |
| 출동 시작 | **Action goal 수락** | `MissionStatus.status = DISPATCHING`, `DeliverAed.Result.started_at` | Action |
| 도착 | **Action 완료** | `MissionStatus.status = ARRIVED`, `DeliverAed.Result.finished_at` | Action |
| 예상 도착 | 이동 중 계속 | `DeliverAed.Feedback.eta_seconds` | Action (feedback) |
| 응답 시간 | 도착 − 접수 | DB 계산 (`response_seconds`) | — |
| 주행 시간 | 도착 − 출동 시작 | DB 계산 (`/api/stats/travel-time`) | — |

응답 시간과 주행 시간을 나눠 재는 이유는, 늦은 원인이 **배정이 늦은
것인지 주행이 느린 것인지** 갈라야 하기 때문이다.

---

## 4. DB 에서 나가는 쪽 — 화면이 쓰는 통신

들어올 때는 ROS 지만 나갈 때는 HTTP 와 WebSocket 이다. 고르는 기준은
ROS 에서와 같다. **계속 흐르는 것은 WebSocket, 한 번 묻고 마는 것은 GET.**

| 경로 | 방식 | 주기 | 무엇을 | 출처 |
|---|---|---|---|---|
| `/ws/live` | WebSocket | 0.25초 | 로봇·현재 이벤트·진행 중 임무·영상 상태 | 메모리(`LiveState`) |
| `/api/live/snapshot` | GET | 요청 시 | 위와 같은 값 한 장 | 메모리 |
| `/api/missions` | GET | 화면이 5초마다 | 최근 임무 요약 | `mission_events` + `mission_assignments` |
| `/api/missions/{id}` | GET | 요청 시 | 그 임무의 상태 전이 전부 | `mission_events` |
| `/api/stats/response-time` | GET | 화면이 5초마다 | 접수→도착 평균·최단·최장 | `emergency_events` + `mission_events` |
| `/api/stats/travel-time` | GET | 요청 시 | 출동→도착 + 계산에 쓴 가정값 | `mission_events` |
| `/api/robots/{id}/track` | GET | 요청 시 | 이동 궤적 | `robot_samples` |
| `/api/video/{stream_id}` | GET (MJPEG) | 연속 | 영상 | 메모리(`FrameBuffer`) |
| `/api/video/{stream_id}/snapshot` | GET | 요청 시 | 한 장 | 메모리 |
| `/api/health` | GET | 요청 시 | 서버·ROS·영상 상태 | 메모리 |

**실시간 값은 DB 를 거치지 않는다.** 로봇 위치는 10Hz 로 오는데 그때마다
쓰고 읽으면 디스크가 병목이 된다. 메모리(`LiveState`)에 두고 0.25초마다
내보내며, DB 에는 1초에 한 번만 남긴다. 화면에서 지금 보이는 숫자와 이력
표의 숫자가 서로 다른 경로로 온다는 뜻이다.

임무 요약(`/api/missions`)은 **저장해 둔 것이 아니라 상태 전이에서 매번
되짚은 것**이다. 요약을 따로 저장하면 이력과 어긋났을 때 어느 쪽을 믿을지
알 수 없다.

---

## 5. 형 변환 규칙

| ROS | SQLite | 어디서 |
|---|---|---|
| `builtin_interfaces/Time` | REAL (UTC epoch 초) | `ros/converters.py: ros_time_to_epoch()` |
| `uint8` 상수 | TEXT (소문자 이름) | `domain/enums.py` |
| `bool` | INTEGER 0/1 | `store/repository.py` |
| `geometry_msgs/PointStamped` | REAL 두 칸 + TEXT `frame_id` | `ros/converters.py` |
| `geometry_msgs/PoseStamped` | REAL 두 칸 + `yaw_deg` | `ros/converters.py: yaw_from_quaternion()` |

**uint8 을 숫자 그대로 넣지 않는 이유**: DB 를 열어 봤을 때 `3` 만 있으면
그게 무슨 상태인지 코드를 봐야 안다. 상수가 추가돼 번호가 밀리면 과거
기록의 뜻이 바뀐다. 이름으로 넣으면 두 문제가 모두 없어진다.

변환은 `domain/enums.py` 한 곳에서만 한다. 화면(TypeScript)도 같은 문자열을
쓰므로, 오타는 컴파일 오류로 드러난다.

**모르는 값이 오면 예외를 낸다.** 조용히 기본값으로 넘기면 잘못된 상태가
DB 에 남고 통계까지 오염된다.

### uint8 ↔ 문자열 전체 표

번호는 `.msg` 의 상수 순서 그대로다. **번호를 중간에 끼워 넣으면 안 된다.**
과거 기록의 뜻이 바뀐다. 새 값은 끝에 붙인다.

| `EmergencyEvent.status` | `MissionStatus.status` | `RobotState.availability` | `RobotState.role` |
|---|---|---|---|
| 0 `detected` | 0 `assigned` | 0 `available` | 0 `none` |
| 1 `confirmed` | 1 `dispatching` | 1 `busy` | 1 `aed_delivery` |
| 2 `dispatched` | 2 `en_route` | 2 `blocked` | 2 `helper_request` |
| 3 `resolved` | 3 `arrived` | 3 `network_lost` | 3 `guide` |
| 4 `canceled` | 4 `completed` | 4 `navigation_error` | 4 `return` |
| | 5 `canceled` | 5 `localization_error` | |
| | 6 `blocked` | 6 `low_battery` | |
| | 7 `network_lost` | 7 `emergency_stop` | |
| | 8 `navigation_error` | 8 `unavailable` | |
| | 9 `recovery_wait` | | |
| | 10 `recovery_resumed` | | |
| | 11 `helper_requested` | | |
| | 12 `helper_en_route` | | |
| | 13 `helper_arrived` | | |

`DeliverAed.Feedback.state` 는 `MissionStatus.status` 와 같은 값을 쓴다.
`DeliverAed.Goal.role` 은 `RobotState.role` 과 같다. 같은 뜻에 다른 번호를
쓰지 않는다.

세 곳(`.msg` 상수 · 파이썬 enum · TypeScript 유니온)이 어긋나지 않았는지는
이렇게 확인한다.

```bash
python3 - <<'PY'
import re, sys
sys.path.insert(0, 'src/aed_hmi')
from backend.domain.enums import MissionState, EventStatus, RobotAvailability, RobotRole

ts = open('src/aed_hmi/frontend/src/types/telemetry.ts', encoding='utf-8').read()
for name, enum in [("MissionState", MissionState), ("EventStatus", EventStatus),
                   ("RobotAvailability", RobotAvailability), ("RobotRole", RobotRole)]:
    block = re.search(rf"type {name}\s*=\s*([^;]+);", ts, re.S).group(1)
    py, front = {m.value for m in enum}, set(re.findall(r"'([^']+)'", block))
    print(f"{name:<20} {'일치' if py == front else f'불일치 {py ^ front}'}")
PY
```

---

## 6. 아직 이어지지 않은 곳

기능 통합 전이라 지금은 끊겨 있다. 리뷰에서 짚어야 할 자리다.

| 자리 | 지금 | 해야 할 일 | 막히는 것 |
|---|---|---|---|
| `location_mapper` | 29줄 뼈대 | 카메라별 이벤트를 `/aed/emergency_event` 로 중계 | **검출이 출동으로 안 이어짐** |
| `robot_state_monitor` | 29줄 뼈대 | `RobotState` 발행 | **`robot_samples` 가 안 쌓임**, 로봇 카드가 빈칸 |
| `event_logger` | 29줄 뼈대 | `MissionStatus` 구독·기록 | 지금은 hmi 가 대신 기록 |
| 좌표 | `vision_detector` 의 `location_x/y` 가 0.0 | 호모그래피로 픽셀→map 변환 | 좌표가 0,0 으로 남음 |
| 카메라 이름 | 노드는 `camera_open`/`camera_alley`, 호모그래피 파일은 `homography_cam1/cam2.yaml` | 이름 맞추기 | 변환식을 못 찾음 |
| 배정 | `MissionAssignment.msg` topic | `DeliverAed` action 으로 교체 | 취소·중복을 손으로 관리 |
| 혼잡도 | `std_msgs/String` | `CrowdLevel.msg` 로 교체 | 판단 근거가 안 따라옴 |
| `crowd_level` | 메시지에는 넣었으나 DB·화면이 안 읽음 | `emergency_events` 에 컬럼 추가 | "왜 2대가 갔나"를 못 되짚음 |
| 속도 | `RobotState.speed_mps` 를 넣었으나 화면이 연속 pose 로 직접 계산 | monitor 가 채우면 그 값 쓰기 | 받는 쪽마다 값이 달라질 수 있음 |

HMI는 실제 `RosBridge`가 수신한 ROS 메시지만 저장하고 화면에 전달한다.
ROS가 연결되지 않으면 가짜 데이터를 만들지 않고 연결 대기 상태를 표시한다.

```
[ROS] RosBridge → context.on_* → repository.insert_* → SQLite
                                                    ↓
      화면 ← /api/missions ← recent_missions() ←────┘
```
