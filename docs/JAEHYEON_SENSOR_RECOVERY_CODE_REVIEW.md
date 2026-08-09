# 박재현 — LiDAR 장애 대응 코드 리뷰 가이드

## 0. 리뷰 범위

담당 범위는 `sensor_recovery` 패키지의 LiDAR 장애 감지와 장애 후 안전 대응이다.

설명은 다음 순서로 진행한다.

```text
LiDAR /scan 감시
-> FAULT 판정
-> Nav2 goal 취소 및 cmd_vel 명령권 인계
-> 장애 직전 목표/경로 보존
-> static map 또는 저장 경로로 fallback 경로 구성
-> odom 기반 위치 추정 + depth 안전 정지
-> 도착 성공 / 실패 후 대체 로봇 요청
-> LiDAR 복구 후 AMCL 재수렴 확인
```

VS Code에서 `[CODE REVIEW]`를 검색하면 아래 설명 순서대로 핵심 코드만 확인할 수 있다.

---

## 1. 디렉토리 구조

```text
src/sensor_recovery/
├── launch/
│   ├── lidar_watchdog.launch.py       # robot별 /scan watchdog
│   └── lidar_fallback.launch.py       # watchdog + fallback response
├── config/
│   ├── lidar_watchdog.yaml            # timeout/recovery 시간
│   └── lidar_fallback.yaml            # 속도/안전/경로 파라미터
├── sensor_recovery/
│   ├── lidar_state_machine.py         # ROS-free LiDAR 생존 상태 머신
│   ├── lidar_watchdog_node.py         # /scan 구독 및 상태 Topic 발행
│   ├── fallback_state_machine.py      # ROS-free fallback 상태 결정
│   ├── fallback_path_follower.py      # 장애 대응 lifecycle과 ROS I/O
│   ├── path_follow_control.py         # 경로 추종/odom/depth 순수 함수
│   ├── grid_path_planner.py           # static map 기반 clearance A*
│   └── replacement_request.py         # 정지 후 대체 요청만 하는 대안 모드
└── test/
    ├── test_lidar_state_machine.py
    ├── test_fallback_state_machine.py
    ├── test_path_follow_control.py
    └── test_grid_path_planner.py
```

핵심 설계 원칙은 ROS callback과 판단 수학을 분리하는 것이다. 시간/센서 값을 입력으로
받는 순수 함수는 ROS2 없이 단위 테스트하고, Node는 Topic/Action/Service 연결을 담당한다.

---

## 2. LiDAR 장애 감지

### `lidar_state_machine.py`

상태 흐름:

```text
STARTING --scan 수신--> ALIVE
STARTING --startup grace 초과--> FAULT
ALIVE --scan timeout--> FAULT
FAULT --scan 재수신--> RECOVERING
RECOVERING --연속 수신 유지--> ALIVE
RECOVERING --다시 timeout--> FAULT
```

`FAULT`에서 scan 한 건을 받았다고 바로 `ALIVE`로 만들지 않는다. 일정 시간 연속으로
scan이 유지되어야 복구로 확정해 센서가 붙었다 끊기는 상황에서 주행 모드가 흔들리지 않는다.

### `lidar_watchdog_node.py`

- robot별 `/robotX/scan`을 SensorData QoS로 구독한다.
- 로봇마다 독립 `LidarMonitor`를 둔다.
- `/robotX/lidar_alive`와 `/robotX/lidar_state`를 발행한다.
- 상태 Topic은 `TRANSIENT_LOCAL`이라 늦게 연결된 노드도 마지막 상태를 즉시 받는다.
- 1초 timeout은 네트워크 지터를 실제 장애로 오인해 실측 후 5초로 조정했다.

---

## 3. Nav2에서 fallback으로 전환

### `fallback_path_follower.py::_on_lidar_state`

`FAULT`로 바뀌는 edge에서 `_start_fallback()`을 한 번 호출한다. 반복 heartbeat는 같은
동작을 다시 시작하지 않는다.

### `fallback_path_follower.py::_start_fallback`

장애 순간에 다음 값을 snapshot한다.

- 마지막 AMCL map pose
- 같은 시점의 odom pose
- 최신 Nav2 `/plan`
- 기존 목적지인 path 마지막 PoseStamped

그다음 NavigateToPose cancel service를 호출하고 즉시 0속도를 발행한다.

### `fallback_path_follower.py::_nav2_cancel_ready`

cancel service가 성공 응답을 줬다는 사실만으로 fallback을 움직이지 않는다. Nav2 Action
status도 active/canceling 상태에서 벗어난 것을 확인한 뒤에만 non-zero `cmd_vel`을 허용한다.
이 지점이 Nav2와 fallback 사이의 속도 명령권 hand-off다.

---

## 4. fallback 경로 생성

### 우선순위 1: 저장된 Nav2 경로

장애 직전 최신 `/plan`에서 현재 위치 이전 구간을 제거하고 남은 경로를 사용한다. 장애가
발생하기 전 Nav2가 이미 주행 가능하다고 계산한 경로이므로 기본 선택으로 사용한다.

### 우선순위 2: static map A*

저장 경로를 쓰지 않는 설정에서는 `grid_path_planner.py`가 latched `/map`만으로 경로를
계산한다. LiDAR 장애 뒤 불안정할 수 있는 live costmap, AMCL TF, Nav2 planner에는 의존하지
않는다.

- `compute_clearance_field()`: 모든 cell에서 가장 가까운 벽까지의 거리 계산
- `plan_path()`: 로봇 반경 + hard margin 안쪽 cell 차단
- soft clearance 구간: 벽에 가까울수록 이동 cost 증가
- `simplify_path()`: 같은 clearance 검사를 통과한 직선 shortcut만 유지

---

## 5. odom 기반 경로 추종과 depth 안전 정지

### 위치 추정

`path_follow_control.py::integrate_odom_delta()`는 마지막 정상 AMCL pose를 map anchor로
사용한다. LiDAR 장애 이후에는 odom의 상대 이동량만 anchor에 합성해 현재 map pose를
추정한다.

### 진행률과 속도 명령

- `update_path_progress()`: 이전 index 주변만 검색해 U자형 경로의 다른 구간으로 점프 방지
- `compute_cmd_vel()`: 목표 방향 오차가 크면 제자리 회전, 방향이 맞으면 전진
- `rate_limit()`: tick 사이 선속도/각속도 급변 제한
- `goal_reached()`: 최종 waypoint와의 거리로 도착 판정

### Depth 안전 판정

`evaluate_depth_safety()`는 한 개의 최소 depth 값 대신 ROI 전체의 유효 픽셀 비율과
근거리 픽셀 비율을 사용한다.

```text
CLEAR              -> 주행 가능
OBSTACLE           -> 즉시 정지
NOISY_DEPTH        -> 즉시 정지
INSUFFICIENT_DATA  -> 설정에 따라 fail-close 또는 제한적 허용
```

짧은 장애물 감지는 `BLOCKED`로 정지 후 재개한다. 설정된 시간을 넘기면 `FAILED`로 전환한다.

---

## 6. fallback 상태와 실패 처리

`fallback_state_machine.py`는 다음 입력만으로 상태를 결정한다.

- plan/odom anchor 존재 여부
- odom stale 여부
- depth 차단 여부와 지속 시간
- stuck 여부
- 경로 이탈 거리
- 도착 여부

```text
IDLE -> STARTING -> ACTIVE <-> BLOCKED -> SUCCEEDED
STARTING/ACTIVE/BLOCKED -> FAILED
```

일시적인 odom 지연이나 짧은 depth 장애물은 `BLOCKED`에서 0속도로 기다린다. plan/anchor
손실, stuck, 과도한 경로 이탈, depth timeout은 스스로 계속 가기 위험해 `FAILED`가 된다.

`FAILED`에서는 다음 Topic을 latched 상태로 발행한다.

- `replacement_needed=True`
- `pending_goal=<기존 목적지>`

---

## 7. LiDAR 복구 처리

LiDAR가 `ALIVE`로 돌아와도 즉시 Nav2를 재개하지 않는다.

1. fallback `cmd_vel`을 0으로 유지한다.
2. fresh/stable AMCL pose를 기다린다.
3. fallback이 이미 도착했다면 위치만 확인하고 기존 goal을 다시 보내지 않는다.
4. 도착 전 복구이고 대체 로봇 요청도 없다면 원래 goal을 Nav2로 재전송한다.
5. 대체 로봇을 요청했다면 기본 설정에서는 중앙 판단을 기다린다.

이미 도착한 goal을 복구 뒤 다시 전송해 로봇이 재출발했던 문제를 이 분기로 방지한다.

---

## 8. 자동 테스트

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=src/sensor_recovery \
pytest src/sensor_recovery/test -q
```

테스트 그룹:

| 대상 | 주요 검증 내용 |
|---|---|
| LiDAR state machine | startup, timeout, recovery, 재장애, clock regression |
| Fallback state machine | ACTIVE/BLOCKED/FAILED/SUCCEEDED 전이와 terminal latch |
| Path follow control | index jump 방지, lookahead, cmd_vel, odom 합성, depth ROI |
| Grid path planner | 벽 margin, unknown cell, corner cut, clearance, simplification |
| Route/corner control | 코너 검출, 감속, 선회 정렬, 남은 거리 |
| Depth/distance utilities | depth decode, 통계, 거리 측정 |

실제 로봇 시험 결과는 `docs/lidar-fallback-summary.md`에 기록되어 있다.

---

## 9. 상위 mission 계층으로 실패 상태 전달

fallback이 안전하게 계속 주행할 수 없으면 다음 두 정보를 latched Topic으로 함께
발행한다.

- `replacement_needed=True`: 현재 로봇이 임무를 계속 수행할 수 없음을 알리는 상태
- `pending_goal=<기존 목적지>`: 다른 로봇이 임무를 이어갈 때 필요한 목적지

LiDAR가 복구되어 상태를 정리할 때는 `replacement_needed=False`를 발행한다. 상태 Topic에
`TRANSIENT_LOCAL` QoS를 사용하므로 상위 mission 계층이 늦게 연결되더라도 마지막 실패
상태와 목적지를 확인할 수 있다.

---

## 10. 실제 리뷰에서 보여줄 핵심 8곳

1. `lidar_state_machine.py::on_tick`
2. `lidar_watchdog_node.py::_handle_transition`
3. `fallback_path_follower.py::_on_lidar_state`
4. `fallback_path_follower.py::_start_fallback`
5. `fallback_path_follower.py::_nav2_cancel_ready`
6. `fallback_path_follower.py::_request_replan`
7. `fallback_path_follower.py::_on_control_tick`
8. `fallback_path_follower.py::_request_replacement`

모든 parameter나 테스트 코드를 줄별로 읽지 않고, 문제 상황과 안전 설계가 연결되는 위
지점만 보여준다.
