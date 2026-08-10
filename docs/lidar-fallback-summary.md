# LiDAR 장애 대응 (`sensor_recovery`) — 전체 정리

박재현 담당 영역인 "LiDAR 장애 감지 및 대응" 관련해서 지금까지 구현한
내용을 한 문서에 정리한다. 시간순 변경 기록은 `jaehyeon-worklog.md`에
있고, 이 문서는 **현재 상태 기준 스냅샷**이다.

## 2026-08-07 현재 자동 전환 흐름

1. Nav2 `NavigateToPose` goal이 ACTIVE인 동안 정상 주행한다.
2. `/scan`이 5초 연속 끊기면 watchdog이 `FAULT`를 발행한다.
3. 마지막 AMCL pose에 이후 odom delta를 합성해 FAULT 판정 순간의 현재
   map pose를 만들고, 원래 goal과 최신 Nav2 global path의 남은 구간을
   저장한다.
4. Nav2 cancel 응답을 확인할 때까지 정지한다. 취소 확인 전에는 fallback이
   non-zero 속도를 발행하지 않는다.
5. 1초 정지 후 저장 경로를 odom feedback으로 추종한다. hard corner는
   감속→정지→제자리 회전→다음 구간 순서로 처리한다.
6. OAK-D `oakd/stereo/image_raw/compressedDepth`를 복원해 전방 ROI를
   검사한다. 장애물/근거리 노이즈는 정지 조건이다. 불안정한 Wi-Fi 때문에
   depth가 없거나 유효 픽셀이 부족하면 경고를 남기고 저속 주행을 계속한다.
7. LiDAR가 3초간 연속 수신돼 `ALIVE`가 되면 fallback을 정지하고, 새 AMCL
   pose를 기다린 뒤 저장한 원래 goal을 Nav2에 다시 보낸다.

fallback 속도 명령은 Nav2와 같은 `cmd_vel_nav` 입력으로 보내고 실제
`cmd_vel`은 velocity smoother 한 노드만 발행한다. watchdog과 controller는
`lidar_fallback.launch.py` 하나로 함께 실행한다.

Nav2 자체의 `controller_server`가 주행 중 "Failed to make progress"로
반복 회전하는 문제는 이 문서 범위에서 제외한다 (다른 팀원이 해결하기로 함,
아래 라이다 장애 대응 로직과는 별개의 DWB/코스트맵 튜닝 이슈).

## 왜 필요한가

로봇이 AED 임무 중 LiDAR가 죽으면:
1. Nav2는 `/scan` 없이는 안전하게 주행할 수 없다 (costmap의
   `obstacle_layer`/`voxel_layer`가 `/scan` 소스, AMCL도 결국 TF가
   stale해짐).
2. 그렇다고 그 자리에 무한정 멈춰있으면 응급 상황 대응이 늦어진다.
3. 두 가지 대응 정책을 둘 다 구현해뒀고, **같은 로봇에 동시에 띄우면 안
   된다** (둘 다 FAULT 시 Nav2 goal을 취소하고 `cmd_vel`을 건드림):
   - `lidar_replacement_request`: 정지 후 "다른 로봇이 가야 함" 신호만
     발행 (기본 선택지, Mission Manager 판단 위임)
   - `lidar_fallback_controller`: LiDAR 없이 정지하지 않고 목적지까지
     자체 주행을 시도 (실험적, 아래에서 자세히 다룸)

## 공통 기반: `lidar_watchdog`

`/robotN/scan` 수신 상태만으로 각 로봇의 LiDAR 상태를 독립적으로
판정한다. 다른 두 노드는 전부 이 노드가 발행하는 `lidar_state`를 구독해서
동작한다.

**상태**: `STARTING → ALIVE ⇄ (FAULT → RECOVERING → ALIVE)`
- `STARTING`: 시작 직후 `startup_grace_sec` 동안은 아직 판정 안 함
- `ALIVE → FAULT`: 마지막 `/scan` 수신 후 `scan_timeout_sec` 경과
- `FAULT → RECOVERING`: `/scan` 다시 들어옴 (아직 "복구 확정"은 아님)
- `RECOVERING → ALIVE`: `recovery_duration_sec` 동안 안정적으로 계속 들어옴
- `RECOVERING` 중 다시 끊기면 즉시 `FAULT`로 되돌아감

**파라미터** (기본값):
| 이름 | 기본값 | 설명 |
|---|---|---|
| `scan_timeout_sec` | `5.0` | 이 시간 넘게 안 들어오면 FAULT (원래 1.0이었으나, 이 랩 환경의 디스커버리 서버 핑이 불안정해서 실제 LiDAR는 멀쩡한데도 1초 넘게 `/scan`이 안 들어오는 경우가 실측됨 → 오탐 방지로 5.0으로 상향) |
| `startup_grace_sec` | `3.0` | 시작 직후 판정 유예 시간 |
| `recovery_duration_sec` | `3.0` | RECOVERING에서 ALIVE로 확정되기까지 안정 유지 시간 |
| `watchdog_period_sec` | `0.1` | 내부 타이머 주기 |
| `status_publish_period_sec` | `1.0` | 전이가 없어도 현재 상태를 재발행하는 heartbeat 주기 |
| `robot_names` | `["robot1", "robot2"]` | 감시할 로봇 목록. `-p robot_names:="['robot2']"`처럼 오버라이드 가능 |
| `scan_topic_suffix` | `scan` | `/robotN/<suffix>` 구독 |

**QoS**: `/scan` 구독은 `qos_profile_sensor_data`(BEST_EFFORT) — 이미
기본으로 best-effort라 추가 변경 불필요함을 확인함. 발행하는
`lidar_state`/`lidar_alive`는 `TRANSIENT_LOCAL`(latched)이라 늦게 붙는
구독자도 현재 상태를 즉시 받음 (단, 구독자도
`--qos-durability transient_local`을 요청해야 함 — 발행 쪽만 latched로는
안 되고 양쪽 다 필요하다는 걸 실측으로 확인함).

**실행**:
```bash
ros2 run sensor_recovery lidar_watchdog --ros-args -p robot_names:="['robot2']"
```

## 정책 A: `lidar_replacement_request` (기본값, Mission Manager 위임)

직접 주행하지 않는다. LiDAR가 죽으면 그냥 멈추고 "이 로봇 대신 다른
로봇이 가야 한다"는 신호만 발행한다.

- `* → FAULT`: `cmd_vel`에 0 Twist, `navigate_to_pose`의 활성 goal 전부
  취소(zero goal_id로 "현재 활성 goal 전체 취소" — 누가 보낸 goal인지
  몰라도 끌 수 있는 표준 방식), 그 순간 `/plan`의 마지막 지점(목적지)을
  저장해서 `pending_goal`로 발행.
- `→ ALIVE`: `replacement_needed`를 false로 되돌리고 **기본적으로 거기서
  끝** — 자동으로 목적지를 재전송하지 않는다. Mission Manager가 그 사이
  다른 로봇으로 재할당했을 수 있어서, 임의로 재출발하면 두 로봇이 같은
  곳으로 동시에 갈 위험이 있기 때문. `auto_resume_on_recovery:=true`로
  켜면 예전처럼 자동 재개(단독 테스트용).

**Run**:
```bash
ros2 run sensor_recovery lidar_replacement_request --ros-args -r __ns:=/robot1
```

## 정책 B: `lidar_fallback_controller` (실험적, LiDAR 없이 자체 주행)

`lidar_state`를 구독해서 Nav2를 대신하고, 자체 상태 머신
(`fallback_state_machine.py`)으로 성공/실패를 명시적으로 판정한다.

```
IDLE → STARTING → ACTIVE ⇄ BLOCKED → SUCCEEDED
(STARTING/ACTIVE/BLOCKED) → FAILED
```

### FAULT 시 흐름

1. 실제 ACTIVE인 `navigate_to_pose` goal이 있는지 확인하고 전체 취소를
   요청한다. cancel 응답 전에는 계속 정지한다.
2. 마지막 `/amcl_pose`에 이후 odom delta를 합성한 현재 pose, `/plan`의
   마지막 지점(목적지), 최신 global path의 남은 구간을 저장한다.
3. **`pre_replan_delay_sec`(기본 1초) 동안 완전 정지** — 즉시 움직이지
   않고 잠깐 멈춰서 상황이 정리될 시간을 준다
4. 기본값은 저장한 Nav2 global path의 현재 위치 이후 구간을 그대로 사용한다.
   저장 경로가 없을 때만 정적 map 자체 A*를 보조 경로로 사용한다.

> **여기서 원래는 Nav2의 `planner_server`(`compute_path_to_pose` 액션,
> `use_start=true`)를 불러서 재계획했었다.** 실기 테스트 결과 "주행 중
> 라이다를 껐을 때 오류가 많이 발생함"이 보고되어 원인을 찾아보니: 그
> global costmap도 `obstacle_layer`가 `/scan` 소스라 LiDAR가 죽으면
> 갱신이 끊기고, AMCL의 `map→odom` TF도 결국 stale해져서 `use_start`로
> AMCL 의존 하나는 피했어도 Nav2 스택 자체가 LiDAR 없이는 성치 않았던 게
> 진짜 원인이었다. 그래서 **Nav2를 완전히 배제하고, 미리 구독해둔 정적
> `/map`(latched)만으로 우리 코드 안에서 직접 경로를 계산하는 방식으로
> 전면 교체**했다. 부수 효과로 액션 왕복 통신이 없어져서 사실상 동기
> 즉시 계산이 되고, `_replanning`/`replan_timeout_sec` 같은 비동기 대기
> 상태 자체가 필요 없어졌다.

### `grid_path_planner.py` — 저장 경로가 없을 때의 보조 경로계산

ROS 비의존 순수 모듈. 사용자 질문 "cmd vel 제어할 때 경로계산도
가능한지" → 가능(정적 지도 + 마지막 pose만 있으면 됨), "최적 경로보다
벽에 안 박게 우선할 수 있는지" → 가능(클리어런스 기반 비용 설계로 구현)
에 대한 답으로 만들어짐.

- `OccupancyGridData`: `nav_msgs/OccupancyGrid`의 ROS-free 미러.
- `compute_clearance_field`: 모든 셀 → 가장 가까운 점유 셀까지 거리를
  멀티소스 BFS로 계산. 맵이 바뀌지 않는 한 맵 수신 시 1회만 계산해서
  캐싱(재계획마다 다시 안 함).
- `plan_path`: 8방향 A*. 비용 설계:
  - `robot_radius_m + hard_margin_m`(기본 0.20 + 0.05 = 0.25m) 이내로
    벽에 붙는 셀은 **통행 자체를 금지**(무한 비용 취급) — "최적 경로보다
    벽에 안 박는 것 우선"이라는 요구사항이 여기 반영됨.
  - 단, **시작 셀만 예외** — 로봇이 고장 시점에 이미 벽 근처에 있었을
    수 있으니 그 자리에서 못 움직이게 되는 걸 막기 위함. 시작 셀에서
    나가는 다음 셀부터는 정상적으로 금지 적용.
  - 벽에서 `soft_clearance_m`(기본 0.4m) 이내인 통행 가능 셀은 비용에
    `1 + wall_clearance_weight * (1 - clearance/soft_clearance_m)`
    (기본 배율 최대 3배)를 곱해서, 여유 있는 통로를 우선 선택하게 함.
  - `allow_unknown_cells`(기본 `false`)가 꺼져 있으면 미탐사(-1) 셀은
    통행 금지 — 안전 우선.
- 경로를 못 찾으면(맵 없음, 시작/목적지가 벽 안, 클리어런스 이내 경로
  없음 등) `None` 반환 → 상태 머신이 `has_plan=False`로 인식해서 자동으로
  실패 처리로 이어짐.
- A* 결과는 `simplify_path`의 grid supercover line-of-sight 검사를 거쳐
  불필요한 직선·zigzag 점을 제거한다. 모든 shortcut 셀이 A*와 동일한
  occupancy 및 hard clearance 조건을 만족해야 하므로 벽이나 안전 여유를
  가로지르지 않는다. 대각선 A* 이동도 두 직교 방향이 모두 통행 가능할
  때만 허용해 벽 모서리 사이를 비집고 통과하지 않는다.

### 제어 틱 (`fallback_control_period_sec`, 기본 0.1초 주기)

1. 저장한 마지막 AMCL pose를 앵커로 `/odom` 델타를 적분해 현재 위치 추정
2. `closest_index`와 `target_index`를 별도로 유지한다. 이전 closest 주변의
   제한된 경로거리 창에서만 새 closest를 찾아 U자형/인접 경로의 먼 미래
   구간으로 점프하지 않게 하고, 평상시 인덱스는 감소시키지 않는다. 기존
   진행점에서 크게 이탈했을 때만 제한된 뒤쪽 구간 재획득을 허용한다.
3. closest부터 경로 누적거리 `lookahead_m` 앞의 target을 선택한다. target
   index도 평상시 감소하지 않으며, 경로 끝까지 남은 거리가 짧으면 마지막
   goal을 선택한다.
4. 목표까지 방위각/거리로 `cmd_vel` 계산 — 각도 오차가
   `linear_heading_threshold_deg`(기본 60도)를 넘으면 직진 성분은 0으로
   고정하고 제자리 회전을 우선한다.
5. 깊이 안전 판단(`evaluate_depth_safety`): 단일 최소값이 아니라 **ROI
   내 픽셀 비율** 기준. 유효 픽셀 부족 → `INSUFFICIENT_DATA`, 가까운 픽셀
   비율이 임계 이상 → `OBSTACLE`. 좌/중앙/
   우 3개 ROI 중 회전 방향에 해당하는 쪽도 같이 확인. `OBSTACLE`과
   `NOISY_DEPTH`는 `cmd_vel`을 0으로 만들고 `BLOCKED`로 전이한다.
   `allow_insufficient_depth_motion=true`이면 `INSUFFICIENT_DATA`는 5초
   주기 경고만 남기고 주행한다. 프레임 나이 기반 판정은 사용하지 않는다.
6. 가속도 제한(`max_linear_accel`/`max_angular_accel`) 적용해 발행

### 실패/성공 판정

- `/odom`이 `odom_timeout_sec` 동안 로컬에 수신되지 않으면 0속도로 정지하고
  `BLOCKED`에서 기다린다. fresh odom 수신 시 `ACTIVE`로 자동 복귀한다.
- `FAILED`(상태별 우선순위): 경로/앵커 없음 →
  정지 판정(`stuck_timeout_sec` 동안 명령은 계속 보냈는데 실제로
  `stuck_distance_m`도 안 움직임) → 경로 이탈
  (`path_deviation_m > max_path_deviation_m`) → 장애물이
  `blocked_timeout_sec`보다 오래 지속. `FAILED`가 되면 **직접**
  `replacement_needed=true` + `pending_goal` 발행(정책 A와 같은 토픽
  계약 재사용, 별도 coordinator 없음).
- `SUCCEEDED`: 현재 위치가 경로 마지막 지점의 `arrival_tolerance_m`
  이내.
- `SUCCEEDED → ALIVE`: 이미 cmd_vel fallback으로 목적지에 도착했으므로
  완료된 Nav2 goal을 다시 보내지 않는다. 로봇을 계속 정지시키고 AMCL
  nomotion update를 요청한다. fresh AMCL pose 3개가 연속 안정 범위에 들면
  `RECOVERY_POSITION_CHECK`로 목표 오차를 기록한 뒤 Nav2는 idle로 유지한다.
- 목적지 도착 전 `→ ALIVE`: 마찬가지로 정지한 상태에서 fresh/stable AMCL을
  기다린 뒤에만 저장 goal로 Nav2를 재개한다. 5초가 지나도 안정 pose가 없으면
  재개를 강행하지 않고 정지 상태로 계속 기다린다.

### 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `max_linear_speed` | `0.20` | 최대 직진 속도(m/s). Nav2 RPP의 `desired_linear_vel`과 동일 |
| `max_angular_speed` | `0.60` | 최대 회전 속도(rad/s). Nav2의 `rotate_to_heading_angular_vel`과 동일 |
| `max_linear_accel` | `0.15` | 직진 가속도 제한 |
| `max_angular_accel` | `0.5` | 회전 가속도 제한 |
| `pre_replan_delay_sec` | `1.0` | FAULT 직후 자체 경로계산 전 정지 유지 시간 |
| `robot_radius_m` | `0.20` | `nav2_aed.yaml`의 `robot_radius`와 일치 — 이 이내로 벽에 붙는 셀은 통행 금지 |
| `hard_margin_m` | `0.05` | `robot_radius_m`에 더하는 고정 안전 여유 |
| `soft_clearance_m` | `0.4` | 이보다 벽에 가까운(통행 가능한) 셀에 비용 페널티 |
| `wall_clearance_weight` | `2.0` | 벽 근접 비용 페널티 강도 |
| `occupied_threshold` | `50` | 이 값 이상 occupancy를 "벽"으로 취급 |
| `allow_unknown_cells` | `false` | true면 미탐사 셀도 통행 허용 |
| `map_topic` | `map` | 정적 지도 구독 토픽 |
| `lookahead_m` | `0.3` | lookahead 거리 |
| `closest_search_ahead_m` | `1.0` | 이전 진행점부터 closest를 찾을 전방 경로거리 창 |
| `closest_search_backtrack_m` | `0.3` | 큰 이탈 시 재획득할 수 있는 후방 경로거리 창 |
| `path_reacquire_distance_m` | `0.5` | 이 거리보다 기존 진행점에서 멀어져야 뒤쪽 재획득 허용 |
| `linear_heading_threshold_deg` | `60.0` | 이 방향 오차를 넘으면 직진을 끄고 회전 우선 |
| `arrival_tolerance_m` | `0.15` | 도착 판정 허용 오차 |
| `min_obstacle_distance_m` | `0.65` | 이보다 가까우면 장애물 후보(실측 stereo 유효 최소거리 반영) |
| `obstacle_pixel_ratio` | `0.03` | ROI 중 이 비율 이상 가까우면 `OBSTACLE` |
| `min_valid_pixel_ratio` | `0.20` | ROI 유효 픽셀이 이보다 적으면 `INSUFFICIENT_DATA` |
| `noise_valid_pixel_ratio` | `0.60` | ROI 유효 픽셀이 이보다 적으면 `NOISY_DEPTH`로 정지 |
| `fallback_control_period_sec` | `0.1` | 제어 틱 주기 |
| `odom_timeout_sec` | `2.0` | 로컬에서 `/odom`을 받지 못한 최대 허용 시간. 초과 시 `FAILED`가 아니라 정지(`BLOCKED`) 후 fresh odom 수신 시 자동 재개 |
| `allow_insufficient_depth_motion` | `true` | depth 없음/유효 픽셀 부족 시 경고 후 저속 fallback 계속(장애물/노이즈는 계속 정지) |
| `blocked_timeout_sec` | `5.0` | `BLOCKED` 지속 허용 시간 |
| `stuck_timeout_sec` | `3.0` | 정지 판정까지의 시간 |
| `stuck_distance_m` | `0.03` | 이보다 안 움직이면 정지 후보 |
| `max_path_deviation_m` | `0.7` | 경로 이탈 허용 거리 |
| `reconvergence_timeout_sec` | `5.0` | ALIVE 후 stable AMCL 미확인 경고 시점(이후에도 정지 유지) |
| `recovery_amcl_required_samples` | `3` | 복구 위치 확정에 필요한 연속 stable AMCL 개수 |
| `recovery_amcl_stability_distance_m` | `0.15` | 연속 AMCL 위치 안정 범위 |
| `recovery_amcl_stability_angle_deg` | `15.0` | 연속 AMCL 방향 안정 범위 |
| `navigate_action` | `navigate_to_pose` | Nav2 주행 액션 이름 |
| `debug_enabled` | `false` | throttled 로그와 RViz용 debug 토픽 활성화 |
| `debug_log_period_sec` | `1.0` | debug 로그/pose/target 발행 주기 |

### Run

```bash
ros2 launch sensor_recovery lidar_fallback.launch.py robot_name:=robot1
```

## 코드 구조 (테스트 가능한 순수 함수 분리)

`lidar_watchdog_node.py`/`replacement_request.py`/`fallback_path_follower.py`는
전부 "구독/타이머/발행만 붙인 얇은 ROS 래퍼"고, 실제 판단 로직은 ROS
비의존 순수 함수로 분리되어 있다 (ROS를 안 띄우고 pytest로 검증 가능):

- `lidar_state_machine.py`: `LidarMonitor`/`LidarState`/`LidarWatchdogConfig`
- `path_follow_control.py`: `update_path_progress`, `find_closest_index`,
  `select_lookahead_target`, `goal_reached`, `path_deviation_m`,
  `compute_cmd_vel`, `rate_limit`, `integrate_odom_delta`,
  `is_stale`, `time_regressed`, `evaluate_depth_safety`, `worst_depth_result`,
  `pose_error`
- `fallback_state_machine.py`: `next_fallback_state`
- `grid_path_planner.py`: `OccupancyGridData`, `compute_clearance_field`,
  `plan_path`, `path_segment_is_safe`, `simplify_path`

## 테스트 현황

- **단위테스트 131개 전부 통과**
  (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/sensor_recovery/test/`):
  회전 오도메트리 합성, yaw ±π 경계, closest/target 분리 및 monotonic 진행,
  U자형 인접 구간 점프 방지, 제한적 경로 재획득, 안전한 경로 단순화, 방향
  오차 기반 회전 우선, 경로 이탈 거리, depth 픽셀비율, 상태 전이, A*(개활지/전체 벽
  차단/부분 벽 우회/좁은 통로 하드 차단/넓은 통로 선호/시작 셀 인플레이션
  예외/미탐사 셀 처리/클리어런스 필드 값·해상도 스케일링) 등.
- **단일 프로세스 통합테스트** (실제 노드를 띄우고 가짜 odom/plan/
  amcl_pose/depth/lidar_state/map을 publish, `/tmp/.../scratchpad/`에
  작성 — 저장소에는 커밋 안 됨):
  - FAULT→`STARTING`→`ACTIVE`(가속도 제한 램프업)→장애물로 `BLOCKED`→
    지속 시 `FAILED`+`replacement_needed`/`pending_goal` 발행→ALIVE 시
    재개 시도 **안 함**(이미 대체 요청함, 게이팅 확인)
  - 이미 목표 근처면 즉시 `SUCCEEDED`, `replacement_needed` 발행 안 됨
  - 자체 경로계산 흐름: 4x4m 가짜 `/map`으로 1초 정지 → 즉시(동기) 경로
    계산 → `STARTING→ACTIVE` 전환, 저속(0.05 m/s) 주행 시작까지 확인
  - `lidar_replacement_request` 두 모드(`auto_resume_on_recovery`
    true/false) 다 확인
- flake8(`--max-line-length=99`), colcon build 전부 clean.

## 실기 확인 결과 (2026-08-08, robot1)

1. 실제 `scan-off`로 Nav2 주행 중 LiDAR를 끄고 5초 뒤 FAULT → Nav2 취소 →
   저장 경로 3.51m를 cmd_vel로 주행 → `SUCCEEDED`까지 확인했다. Depth
   정지/0.5초 clear hold도 3회 정상 동작했다.
2. 당시 LiDAR 복구 후에는 fallback이 이미 도착했는데도 기존 goal을 다시
   보내는 문제를 발견했다. 현재 코드는 stable AMCL 위치만 확인하고 Nav2를
   idle로 유지하도록 분기해 완료 goal의 재전송을 차단한다.

운용 환경에 따라 실제 map 크기에서 A*/클리어런스 계산 시간과
`soft_clearance_m`/`wall_clearance_weight`/`robot_radius_m`은 조정할 수 있다.
직접 주행 대신 정지·대체 요청만 필요한 정책은 `lidar_replacement_request`로
분리했으며, 두 동작 모드는 단일 프로세스 통합 테스트로 검증했다.

## 인프라 도구

- **`tools/lidar_toggle.sh`**: 실제 rplidar 드라이버를 SSH로 kill/재실행
  (`scan-off`/`scan-on`, robot1/robot2만 지원). 모터만 정지시키는
  `stop`/`start`는 이 로봇 펌웨어에서 `/scan`이 계속 발행돼 fault 재현이
  안 돼서 별도로 만듦. 주행 중(도킹 해제 상태)에 fault를 재현하는 게
  `lidar_fallback_controller`를 검증하는 목적 자체라, 기본 도킹 체크를
  의도적으로 우회하는 `--allow-undocked` 플래그 추가함(기본값은 여전히
  거부, 명시적 플래그로만 우회).
  ```bash
  tools/lidar_toggle.sh scan-off 2 --yes --allow-undocked
  tools/lidar_toggle.sh scan-on 2
  ```
- **`tools/test_lidar_fault_cycle.sh`**: 두 번째 터미널에서 Enter 두 번으로
  주행 중 `scan-off`와 fallback 도착 후 `scan-on`만 순서대로 실행한다.
- **`tools/aliases.sh`**: `mapnav`(지도+AMCL+Nav2+RViz 한 번에, 로그
  파일로도 저장하며 watchdog/fallback 기본 포함), `initpose`,
  `pf`(preflight 점검), `dock`/`undock` 등.

## 상위 계층 연동 인터페이스

watchdog은 `lidar_alive`/`lidar_state`, fallback은 `fallback_state`와
`replacement_needed`/`pending_goal`을 발행한다. 센서 판정과 주행 대응을 이
토픽 계약으로 분리했기 때문에 RobotState 또는 Mission Manager는 내부 제어
로직에 의존하지 않고 필요한 상태만 집계할 수 있다.
