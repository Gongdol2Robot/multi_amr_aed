# RobotState LiDAR 필드 연동 제안 (미구현, 협의 대기)

작성: 박재현. `aed_interfaces`(이현민 담당), `mission_manager`(김재엽 담당)
쪽 코드는 건드리지 않았음 — 아래 내용 협의/확정되면 한 번에 구현한다.

## 문제

`lidar_watchdog`(`sensor_recovery` 패키지, 구현 완료)이 로봇별 LiDAR 상태
(`STARTING`/`ALIVE`/`FAULT`/`RECOVERING`)를 판정해서 `lidar_alive`,
`lidar_state` 토픽으로 발행하고 있다. 하지만 이건 Mission Manager가 보는
공식 상태 메시지(`aed_interfaces/msg/RobotState`, `/aed/robot_state` 토픽)와
완전히 분리되어 있다.

`RobotState.msg`(현재 정의, 수정 전):

```text
uint8 AVAILABLE=0
uint8 BUSY=1
uint8 BLOCKED=2
uint8 NETWORK_LOST=3
uint8 NAVIGATION_ERROR=4
uint8 LOCALIZATION_ERROR=5
uint8 LOW_BATTERY=6
uint8 EMERGENCY_STOP=7
uint8 UNAVAILABLE=8

uint8 ROLE_NONE=0
uint8 ROLE_AED_DELIVERY=1
uint8 ROLE_HELPER_REQUEST=2
uint8 ROLE_GUIDE=3
uint8 ROLE_RETURN=4

string robot_id
builtin_interfaces/Time stamp
geometry_msgs/PoseStamped pose
float32 battery_percentage
uint8 availability
uint8 role
string mission_id
bool is_docked
bool network_ok
bool localization_ok
bool nav2_ok
bool emergency_stop
bool path_valid
float32 estimated_path_cost
builtin_interfaces/Time last_heartbeat
string detail
```

`network_ok`/`localization_ok`/`nav2_ok`/`emergency_stop`은 있는데 LiDAR
상태를 담을 필드가 없다. `mission_manager/manager_node.py`의
`_is_available()`도 이 필드들만 검사하고 LiDAR는 아예 보지 않는다:

```python
@staticmethod
def _is_available(state: RobotState) -> bool:
    return (
        state.availability == RobotState.AVAILABLE
        and state.network_ok
        and state.localization_ok
        and state.nav2_ok
        and not state.emergency_stop
        and bool(state.pose.header.frame_id)
    )
```

결과적으로 LiDAR가 FAULT여도 Mission Manager는 그 로봇을 여전히 PRIMARY
후보로 선택할 수 있다.

## 제안: `RobotState.msg`에 필드 1개 추가

기존 `*_ok` bool 필드들과 동일한 스타일로 맞춘다 (새 enum이나 구조체 없이
최소 변경):

```diff
 bool is_docked
 bool network_ok
 bool localization_ok
 bool nav2_ok
+bool lidar_ok
 bool emergency_stop
 bool path_valid
 float32 estimated_path_cost
 builtin_interfaces/Time last_heartbeat
 string detail
```

- `lidar_ok`: watchdog 상태가 `ALIVE`일 때만 `true`. `STARTING`/`FAULT`/
  `RECOVERING`은 전부 `false` (아직 안전 주행을 보장할 수 없는 상태로 취급).
- 장애 원인 문자열은 새 필드 없이 기존 `detail`을 재사용한다. 예:
  `detail = "LIDAR_TIMEOUT"` (FAULT), `detail = "LIDAR_RECOVERING"`
  (RECOVERING).
- `fault_code`, `last_scan_age` 같은 별도 필드는 만들지 않는다 — 지금
  Mission Manager가 판단에 쓰는 건 불리언 게이트뿐이고, 세부 진단 정보는
  HMI 쪽에서 필요하면 `sensor_recovery`가 별도로 발행하는 `lidar_state`
  토픽(`STARTING`/`ALIVE`/`FAULT`/`RECOVERING`)을 그대로 구독하면 된다.

## 연동 파이프라인

```text
sensor_recovery/lidar_watchdog          (구현 완료, 박재현)
  → /robotN/lidar_alive, /robotN/lidar_state
        │
        ▼
robot_state_monitor                     (scaffold만 존재, 박재현 담당)
  → lidar_alive/lidar_state 구독, 다른 상태(배터리/localization/nav2)와
    합쳐 RobotState.lidar_ok, RobotState.detail 채워서 발행
        │
        ▼
/aed/robot_state (RobotState)
        │
        ▼
mission_manager._is_available()         (김재엽 담당)
  → `and state.lidar_ok` 한 줄 추가
```

## 수정이 필요한 파일 (구현 시점 기준, 지금은 미수정)

| 파일 | 담당 | 변경 내용 |
|---|---|---|
| `src/aed_interfaces/msg/RobotState.msg` | 이현민 | `bool lidar_ok` 필드 추가 |
| `src/robot_state_monitor/robot_state_monitor/robot_state_monitor.py` | 박재현 | `lidar_alive`/`lidar_state` 구독 → `RobotState.lidar_ok`/`detail` 반영 후 `/aed/robot_state` 발행 |
| `src/mission_manager/mission_manager/manager_node.py` | 김재엽 | `_is_available()`에 `and state.lidar_ok` 추가 |

메시지 스키마를 바꾸는 파일이 이현민 담당이라, `RobotState.msg` 변경은
협의 후 이현민이 직접 반영하거나 PR 리뷰를 받는 게 맞다. 나머지 두 개는
박재현/김재엽 각자 담당 파일이라 별도 조율 없이 진행 가능.

## 열린 질문 (팀 협의 필요)

1. `RECOVERING` 상태를 `lidar_ok=false`로 유지할지, 아니면 `true`로 조기
   전환할지 — 지금 제안은 `ALIVE`만 `true` (보수적).
2. `robot_state_monitor`가 다른 상태 소스(배터리, AMCL, Nav2)를 아직
   구현 전이라, LiDAR 필드만 채우고 나머지는 scaffold 로그만 찍는 부분
   구현으로 먼저 갈지, 전체 완성 후 합칠지.
3. HMI(`aed_hmi`)가 `lidar_ok` 대신 원본 `lidar_state`(4단계)를 직접
   보여주고 싶어할 수 있음 — `lidar_state` 토픽은 계속 유지되니 HMI는
   `RobotState`가 아니라 이걸 직접 구독하면 됨.
