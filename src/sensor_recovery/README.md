# sensor_recovery

담당: 박재현

LiDAR `/scan` 수신 상태를 감시하고, 장애 시 Nav2 제어권을 안전하게 인계받아
저장 경로·odom·OAK-D depth로 대응 주행을 수행합니다.

## Nodes

- `lidar_watchdog` (구현 완료): 로봇별 `/scan` timeout·복구를 독립적으로 판정하고
  `lidar_alive`/`lidar_state`를 발행. 아래 참고.
- `lidar_fallback_controller` (구현 완료, 상태 머신 포함): LiDAR FAULT 시 Nav2를
  멈추고 오도메트리+깊이카메라로 직접 주행. `IDLE/STARTING/ACTIVE/BLOCKED/
  SUCCEEDED/FAILED` 상태를 명시적으로 발행하고, `FAILED`가 되면 대체 로봇을
  직접 요청한다. 아래 참고. **지금은 `lidar_replacement_request`와 동시에
  실행하지 않는다.**
- `lidar_replacement_request` (구현 완료): LiDAR FAULT 시 주행만 멈추고
  대체 로봇이 필요하다는 신호를 발행 (자동으로 다른 로봇에 보내지 않음 —
  사람 또는 이후 Mission Manager가 판단). 아래 참고.
- `cmd_vel_distance_test` (거리 보정 시험): 정해진 시작 위치에서
  경로 추종 없이 일정한 `cmd_vel`만 10초간 발행해 0.5m 이동시키고,
  AMCL의 예상 위치와 실제 위치 오차를 계산. 아래 참고.
- `sensor_health_monitor`: 패키지 호환 entry point. 실제 장애 감지와 대응은 위
  watchdog/fallback 노드가 담당한다.

핵심 산출물은 LiDAR Watchdog, Nav2-fallback 전환 제어, 장애 주입 시험 결과입니다.

## lidar_watchdog

로봇별 `/scan` 수신 시각을 독립적으로 추적해 `STARTING → ALIVE → FAULT → RECOVERING`
상태를 판정합니다. 상태 전이 로직은 `sensor_recovery/lidar_state_machine.py`에
ROS와 무관한 순수 클래스(`LidarMonitor`)로 분리되어 있고, `lidar_watchdog_node.py`는
그 위에 구독·타이머·발행만 붙인 얇은 래퍼입니다.

### Parameters (`config/lidar_watchdog.yaml`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `scan_timeout_sec` | `5.0` | 마지막 수신 후 이 시간이 지나면 FAULT (네트워크 지터로 인한 순간적 끊김에 오탐하지 않도록 여유를 둠) |
| `startup_grace_sec` | `3.0` | 시작 직후 FAULT 판정을 유예하는 시간 |
| `recovery_duration_sec` | `3.0` | RECOVERING에서 ALIVE로 복귀하기 위한 연속 정상 수신 시간 |
| `watchdog_period_sec` | `0.1` | 타임아웃 재평가 타이머 주기 |
| `status_publish_period_sec` | `1.0` | 전이가 없어도 현재 상태를 다시 발행하는 주기 |
| `robot_names` | `[robot1, robot2]` | 감시 대상 로봇 |
| `scan_topic_suffix` | `scan` | `/{robot_name}/{scan_topic_suffix}` 형태로 구독 |

### Topics

- 구독: `/robot1/scan`, `/robot2/scan` (`sensor_msgs/msg/LaserScan`)
- 발행: `/robot1/lidar_alive`, `/robot2/lidar_alive` (`std_msgs/msg/Bool`)
- 발행: `/robot1/lidar_state`, `/robot2/lidar_state` (`std_msgs/msg/String`,
  값은 `STARTING`/`ALIVE`/`FAULT`/`RECOVERING`)

전이가 발생하면 즉시 발행하고, 전이가 없어도 `status_publish_period_sec`
(기본 1초)마다 현재 상태를 다시 발행합니다. 로그는 여전히 전이가 실제로
일어날 때만 찍습니다(반복 로그 방지). QoS durability는 `TRANSIENT_LOCAL`
(latched)이라, 늦게 구독해도(같은 durability를 요청하면) 최대 1초 안에는
현재 상태를 받을 수 있습니다. 기본 volatile로 구독하는 `ros2 topic echo`는
latched 값 자체는 못 받지만, 1초 주기 재발행 덕분에 곧 다음 값이 옵니다.
latched 값을 즉시 받고 싶다면:

```bash
ros2 topic echo /robot2/lidar_state --qos-durability transient_local
```

LiDAR 상태 계약은 `lidar_alive`와 `lidar_state` 전용 토픽으로 분리했다. 이 구조는
`RobotState` 메시지 개정 없이도 watchdog, fallback, 관제 계층이 같은 상태를
구독할 수 있고, 필요하면 상위 계층에서 RobotState로 집계할 수 있다.

### Run

```bash
ros2 launch sensor_recovery lidar_watchdog.launch.py
# 또는
ros2 run sensor_recovery lidar_watchdog
```

### 실제 로봇 LiDAR 껐다 켜기 (테스트용)

`rplidar_ros`가 제공하는 `/robotN/stop_motor`, `/robotN/start_motor`
(`std_srvs/srv/Empty`) 서비스로 실제 LiDAR 모터를 안전하게 끄고 켤 수 있습니다.
드라이버 프로세스를 죽이거나 하드웨어를 직접 만지지 않는 공식 인터페이스입니다.
`tools/lidar_toggle.sh`로 래핑되어 있고, 정지 전에 로봇이 도킹 상태인지
확인하고 사용자 확인을 받습니다(다른 팀원이 쓰고 있을 수 있으므로).

```bash
tools/lidar_toggle.sh status 1   # 서비스 존재만 확인 (read-only)
tools/lidar_toggle.sh stop 1     # robot1 LiDAR 모터만 정지 (도킹 확인 + 확인 프롬프트)
tools/lidar_toggle.sh start 1    # robot1 LiDAR 모터 재시작
```

**주의**: 이 로봇 펌웨어 기준으로 `stop_motor`는 모터 회전만 멈추고 드라이버는
계속 떠 있어서, 멈춘 자리의 값을 고정으로 계속 발행합니다 — `/scan`이
끊기지 않으므로 watchdog의 FAULT 판정을 재현하지 못합니다(실측 확인함).
`/scan`을 실제로 끊어서 FAULT/RECOVERING을 재현하려면 `scan-off`/`scan-on`을
쓰세요. SSH로 `rplidar_composition` 드라이버 프로세스 자체를 kill/재실행합니다
(이 launch에는 respawn이 없어 kill하면 자동으로 안 살아남 — 확인함). 다른
노드(base, oakd, joy 등)에는 영향 없습니다.

```bash
tools/lidar_toggle.sh scan-off 1     # robot1 LiDAR 드라이버 kill → /scan 완전히 끊김
tools/lidar_toggle.sh scan-on 1      # robot1 LiDAR 드라이버 재실행
```

SSH 접속 정보는 저장소에 커밋된 `.env.robots`에서 읽는다.

### Extension points

상태가 FAULT/복구로 바뀔 때 `handle_lidar_fault(robot_name)` /
`handle_lidar_recovery(robot_name)` hook이 호출된다. watchdog은 상태 판정과 발행에
집중하고, Nav2 Goal 취소·안전 정지·대체 요청은 별도
`lidar_fallback_controller`/`lidar_replacement_request`가 담당한다.

## lidar_fallback_controller

`lidar_state`(watchdog이 발행)를 구독해서 Nav2를 대신하고, 자체 상태 머신
(`fallback_state_machine.py`)으로 성공/실패를 명시적으로 판정합니다.

```text
IDLE → STARTING → ACTIVE ⇄ BLOCKED → SUCCEEDED
(STARTING/ACTIVE/BLOCKED) → FAILED
```

- `* → FAULT`: 실제 ACTIVE인 `navigate_to_pose` goal이 있을 때만 동작한다.
  마지막 AMCL pose에 이후 odom delta를 합성한 현재 pose, 원래 목적지,
  최신 Nav2 global path의 현재 위치 이후 구간을 저장하고 goal 취소를
  요청한다. cancel 응답 확인 전에는 0 명령만 유지한다. 확인 후에도
  `pre_replan_delay_sec`(기본 1초) 동안 정지한 다음 저장 경로를 추종한다.
  저장 경로가 없을 때만 정적 map 자체 A*를 보조 경로로 사용한다.
- A* grid 경로는 `simplify_path`의 wall-clear line-of-sight 검사로 직선상의
  불필요한 점과 작은 zigzag를 제거한다. shortcut이 닿는 모든 grid cell이
  A*와 같은 occupancy/hard-clearance 조건을 통과해야 하므로 벽 안전 여유는
  유지된다. A* 자체도 막힌 두 셀 사이 대각선 corner-cutting을 금지한다.
- 매 제어 틱(`fallback_control_period_sec`): 마지막 AMCL pose 기준 `/odom`
  델타를 적분해 현재 위치 추정 → 별도 `closest_index`/`target_index` 갱신.
  closest는 이전 진행점 주변의 제한된 경로거리 창에서만 검색해 U자형 경로의
  가까운 미래 구간으로 점프하지 않게 하고, 큰 이탈일 때만 제한된 후방
  재획득을 허용한다. closest부터 누적거리 기준 lookahead target을 선택한 뒤
  `cmd_vel` 계산(기본 30도 이상 방향 오차는 직진 0, 회전 우선) → 가속도
  제한을 적용한다. hard corner는 감속→정지→제자리 정렬 뒤 다음 구간으로
  넘어간다. 명령은 `cmd_vel_nav`로 보내 실제 `cmd_vel` 출력은 Nav2 velocity
  smoother 하나만 담당한다.
- 깊이 안전 판단(`evaluate_depth_safety`)은 단일 최소값이 아니라 **ROI 내
  픽셀 비율** 기준입니다: 유효 픽셀이 `min_valid_pixel_ratio` 미만이면
  `INSUFFICIENT_DATA`, `min_obstacle_distance_m`보다 가까운 픽셀이
  `obstacle_pixel_ratio` 이상이면 `OBSTACLE`. 전방을 좌/중앙/우 3개 ROI로
  나누고, 계산된 회전 방향에 따라 회전하는 쪽 ROI도 같이 확인합니다.
  프레임 나이 기반 판정은 사용하지 않습니다. `OBSTACLE`과 `NOISY_DEPTH`는
  `cmd_vel`을 0으로 만들고 상태를 `BLOCKED`로 전이합니다. 현재 실제 로봇
  설정은 불안정한 Wi-Fi를 고려해 `allow_insufficient_depth_motion=true`이며,
  `INSUFFICIENT_DATA`는 경고만 남기고 저속 fallback을 계속합니다.
- `/odom`이 `odom_timeout_sec` 동안 로컬에 수신되지 않으면 즉시 0속도를
  발행하고 `BLOCKED`에서 기다린다. fresh odom이 들어오면 자동으로 `ACTIVE`로
  복귀하며, 일시적인 네트워크 지연만으로 `FAILED` 처리하지 않는다.
- 실패 판정(→ `FAILED`, 상태별 우선순위대로): 저장된 경로/앵커 없음,
  정지 판정(`stuck_timeout_sec` 동안
  cmd_vel은 계속 보냈는데 실제로는 `stuck_distance_m`도 안 움직임),
  경로 이탈(`path_deviation_m` > `max_path_deviation_m`), 장애물이
  `blocked_timeout_sec`보다 오래 지속. `FAILED`가 되면 **직접**
  `replacement_needed=true` + `pending_goal`을 발행합니다(별도 coordinator
  없이 `lidar_replacement_request`와 같은 토픽 계약 재사용).
- 도착 판정(현재 위치가 경로 마지막 지점의 `arrival_tolerance_m` 이내) →
  `SUCCEEDED`.
- `SUCCEEDED → ALIVE`: 이미 fallback으로 목적지에 도착했으므로 완료된 Nav2
  goal을 다시 보내지 않는다. 정지 상태에서 AMCL nomotion update를 요청하고
  fresh/stable AMCL pose 3개를 확인한 뒤 `RECOVERY_POSITION_CHECK`로 목표
  오차만 기록한다.
- 목적지 도착 전 `→ ALIVE`: stable AMCL을 확인한 뒤에만 저장 goal로 Nav2를
  재개한다. 5초가 지나도 확인되지 않으면 재개를 강행하지 않고 계속 정지한다.

제어/판정 로직은 전부 `sensor_recovery/path_follow_control.py`
(`update_path_progress`, `find_closest_index`, `select_lookahead_target`,
`goal_reached`, `path_deviation_m`,
`compute_cmd_vel`, `rate_limit`, `integrate_odom_delta`, `is_stale`,
`time_regressed`, `evaluate_depth_safety`, `worst_depth_result`, `pose_error`),
`sensor_recovery/fallback_state_machine.py`(`next_fallback_state`), 그리고
`sensor_recovery/grid_path_planner.py`(경로 계산)에 ROS 비의존 순수 함수로
분리되어 있습니다. `fallback_path_follower.py`는 그 위에 구독/타이머/발행만
붙인 래퍼입니다 (`lidar_watchdog`와 동일 구조).

**`grid_path_planner.py`**: `OccupancyGridData`(ROS `OccupancyGrid` 메시지의
ROS-free 미러) + `compute_clearance_field`(모든 셀에서 가장 가까운 점유
셀까지의 거리를 BFS로 계산, 맵이 바뀌지 않는 한 한 번만 계산해서 재사용) +
`plan_path`(8방향 A*). 비용 설계:
- `robot_radius_m + hard_margin_m` 이내로 벽에 붙는 셀은 **통행 자체를
  금지**(값 자체가 무한대 취급) — 최적 경로보다 "벽에 안 박는 것"을
  우선한다는 요구사항이 여기 반영됨.
- 다만 시작 셀 자체는 이 금지에서 예외(로봇이 고장 시점에 이미 벽에
  가까이 있었을 수 있으니 그 자리에서 못 움직이는 걸 막기 위함) — 시작
  셀에서 나가는 다음 셀부터는 정상적으로 금지 적용.
- 벽에서 `soft_clearance_m` 이내인(통행은 가능한) 셀은 거리 비용에
  `1 + wall_clearance_weight * (1 - clearance/soft_clearance_m)` 배율을 곱해
  더 비싸게 만든다 — 여유 있는 통로가 있으면 그쪽을 우선 선택하게 됨.
- `allow_unknown_cells`(기본 `false`)가 꺼져 있으면 미탐사(-1) 셀은 통행
  금지(미지 영역을 가로지르는 것보다 안전을 우선).

### Parameters

| 이름 | 기본값 | 설명 |
|---|---|---|
| `max_linear_speed` | `0.20` | 최대 직진 속도(m/s). Nav2 RPP의 `desired_linear_vel`과 동일 |
| `max_angular_speed` | `0.60` | 최대 회전 속도(rad/s). Nav2의 `rotate_to_heading_angular_vel`과 동일 |
| `max_linear_accel` | `0.15` | 직진 가속도 제한 (m/s²) |
| `max_angular_accel` | `0.5` | 회전 가속도 제한 (rad/s²) |
| `pre_replan_delay_sec` | `1.0` | FAULT 직후 자체 경로계산 전 정지 유지 시간 |
| `robot_radius_m` | `0.20` | 로봇 반경 (`nav2_aed.yaml`의 `robot_radius`와 일치) — 이 이내로 벽에 붙는 셀은 통행 금지 |
| `hard_margin_m` | `0.05` | `robot_radius_m`에 더하는 고정 안전 여유 |
| `soft_clearance_m` | `0.4` | 이보다 벽에 가까운(그러나 통행 가능한) 셀에 비용 페널티 부여 |
| `wall_clearance_weight` | `2.0` | 벽 근접 비용 페널티 강도 (배율 상한 `1+`이 값) |
| `occupied_threshold` | `50` | 이 값 이상의 occupancy를 "벽"으로 취급 |
| `allow_unknown_cells` | `false` | `true`면 미탐사(-1) 셀도 통행 허용 |
| `map_topic` | `map` | 정적 지도를 구독할 토픽 이름 |
| `lookahead_m` | `0.2` | lookahead 거리 |
| `closest_search_ahead_m` | `1.0` | closest 검색의 전방 경로거리 창 |
| `closest_search_backtrack_m` | `0.3` | 큰 이탈 시 재획득 가능한 후방 경로거리 창 |
| `path_reacquire_distance_m` | `0.5` | 기존 진행점에서 이 거리 이상 이탈해야 후방 재획득 허용 |
| `linear_heading_threshold_deg` | `30.0` | 이 방향 오차를 넘으면 직진 0, 회전 우선 |
| `arrival_tolerance_m` | `0.15` | 도착 판정 허용 오차 |
| `min_obstacle_distance_m` | `0.65` | 이보다 가까우면 장애물 후보(실측 stereo 유효 최소거리 반영) |
| `obstacle_pixel_ratio` | `0.03` | ROI 중 이 비율 이상 가까우면 `OBSTACLE` |
| `min_valid_pixel_ratio` | `0.20` | ROI 유효 픽셀이 이보다 적으면 `INSUFFICIENT_DATA` |
| `noise_valid_pixel_ratio` | `0.60` | ROI 유효 픽셀이 이보다 적으면 `NOISY_DEPTH`로 정지 |
| `fallback_control_period_sec` | `0.1` | 제어 틱 주기 |
| `odom_timeout_sec` | `2.0` | 로컬에서 `/odom`을 받지 못한 최대 허용 시간. 초과 시 `FAILED`가 아니라 정지(`BLOCKED`) 후 fresh odom 수신 시 자동 재개 |
| `allow_insufficient_depth_motion` | `true` | depth 없음/유효 픽셀 부족 시 경고 후 주행 계속(장애물/노이즈는 계속 정지) |
| `blocked_timeout_sec` | `5.0` | `BLOCKED` 지속 허용 시간 |
| `stuck_timeout_sec` | `3.0` | 정지 판정까지의 시간 |
| `stuck_distance_m` | `0.03` | 이보다 안 움직이면 정지 후보 |
| `max_path_deviation_m` | `0.7` | 경로 이탈 허용 거리 |
| `reconvergence_timeout_sec` | `5.0` | ALIVE 후 stable AMCL 미확인 경고 시점(계속 정지) |
| `recovery_amcl_required_samples` | `3` | 복구 위치 확정에 필요한 연속 stable AMCL 개수 |
| `recovery_amcl_stability_distance_m` | `0.15` | 연속 AMCL 위치 안정 범위 |
| `recovery_amcl_stability_angle_deg` | `15.0` | 연속 AMCL 방향 안정 범위 |
| `navigate_action` | `navigate_to_pose` | Nav2 주행 액션 이름 |
| `debug_enabled` | `false` | throttled 로그와 RViz debug 토픽 활성화 |
| `debug_log_period_sec` | `1.0` | debug 정보 발행 주기 |

### Topics/Action (네임스페이스 리맵 전제, `robot_missions/mission_executor`와
동일 방식: `-r __ns:=/robot1`로 실행)

- 구독: `lidar_state`(String), `plan`(nav_msgs/Path, 목적지 추출용), `odom`
  (nav_msgs/Odometry), `amcl_pose`(PoseWithCovarianceStamped),
  `oakd/stereo/image_raw/compressedDepth`(sensor_msgs/CompressedImage),
  `map`(nav_msgs/OccupancyGrid,
  `TRANSIENT_LOCAL` — map_server가 발행하는 정적 지도를 캐싱해서 FAULT 시
  자체 경로계산에 사용; 늦게 시작해도 latched라 마지막 값을 바로 받음)
- 발행: `cmd_vel_nav`(Twist), `fallback_state`(String, `TRANSIENT_LOCAL`,
  `IDLE`/`STARTING`/`ACTIVE`/`BLOCKED`/`SUCCEEDED`/`FAILED`),
  `replacement_needed`(Bool, `TRANSIENT_LOCAL`, `lidar_replacement_request`와
  동일 계약), `pending_goal`(PoseStamped, `TRANSIENT_LOCAL`). debug 활성 시
  `fallback_debug/path`(Path), `fallback_debug/target`(PoseStamped),
  `fallback_debug/estimated_pose`(PoseStamped)도 발행.
- 액션 클라이언트:
  - `navigate_to_pose` — 취소는 `<action>/_action/cancel_goal` 서비스에 빈
    goal_id로 요청("현재 활성 goal 전체 취소"의 표준 방식이라 누가 보낸
    goal인지 몰라도 끌 수 있음), 복구 시 재개 goal 전송용. **경로 재계산은
    더 이상 Nav2 액션을 쓰지 않는다** — `grid_path_planner.py`로 자체
    계산(위 참고).

### Run

```bash
ros2 launch sensor_recovery lidar_fallback.launch.py robot_name:=robot1
```

통합 launch는 실기 분석을 위해 `debug_enabled: true`로 실행하므로 throttled
로그와 `fallback_debug/*` RViz 토픽을 함께 확인할 수 있다.

### 검증 상태

- `path_follow_control.py` + `fallback_state_machine.py` + `grid_path_planner.py`
  + `route_test_support.py` + `distance_test_metrics.py` 순수 함수 단위테스트
  131개 통과(회전 오도메트리
  합성, yaw가 ±π를 넘는
  경우, closest/target index 분리, U자형 구간 점프 방지, 제한적 재획득,
  wall-clear 경로 단순화, 회전 우선 제어, 경로 이탈 거리, depth 픽셀비율,
  상태 전이 규칙, A* 개활지/부분 벽 우회/전체 벽 차단/좁은 통로 하드 차단/
  넓은 통로 선호/시작 셀 인플레이션 예외/미탐사 셀 처리/클리어런스 필드
  값 등).
- 실제 노드를 그대로 띄우고 가짜 odom/plan/amcl_pose/depth/lidar_state/map을
  publish하는 단일 프로세스 통합 테스트 여러 종으로 확인:
  - FAULT→`STARTING`→`ACTIVE`(가속도 제한 램프업 확인)→장애물로
    `BLOCKED`→지속 시 `FAILED`+`replacement_needed`/`pending_goal`
    발행→ALIVE 시 재개 시도 **안 함**(이미 대체 요청함, 게이팅 확인).
  - 이미 목표 근처인 경우 즉시 `SUCCEEDED`, `replacement_needed` 발행 안 됨.
  - **자체 경로계산 흐름**: 4x4 m 개활 지도를 가짜로 publish해서 확인 —
    FAULT 직후 `pre_replan_delay_sec`(1초) 동안 `cmd_vel` 전부 0 유지, 그
    다음 Nav2 액션 왕복 없이 그 자리에서 바로(동기) 경로가 계산되어
    `STARTING→ACTIVE` 전환 및 저속(0.05 m/s) 주행 시작까지 확인.
- robot1에서 실제 LiDAR driver를 끄고 5초 뒤 FAULT → Nav2 취소 → 저장 경로
  3.51m fallback 주행 → `SUCCEEDED`까지 확인했다. 주행 중 OAK-D depth 정지와
  0.5초 clear hold도 3회 정상 동작했다. 수동 takeover 4.35m 시험에서 측정한
  odom-AMCL 횡오차는 robot1 전용 odom 보정값에 반영했다.

## cmd_vel_distance_test (0.5m 기준 보정 시험)

기존 fallback/경로 추종 코드는 그대로 보존하되, 문제를 가장 작은 단위부터
다시 검증하기 위해 새로 만든 독립 노드다. 이 시험에는 경로, A*, lookahead,
depth, `/scan`, LiDAR 장애 상태 머신이 전혀 개입하지 않는다.

시험 순서는 다음과 같다.

1. 실행 스크립트가 시험 노드를 수동 대기 모드로 먼저 띄운다. 그다음 Nav2로
   명목 시작 pose `(0.80, 0.20, 90deg)`에 로봇을 배치한다. 시험 노드는 이
   이동 중 발행되는 AMCL pose를 저장한다.
2. Enter를 누르면 스크립트가 내부에서 start 서비스를 호출한다. 사용자가
   별도 명령을 입력할 필요는 없다. 노드는 활성 Nav2 goal을 취소하고 1초간
   0 속도를 유지한다.
3. 이 시점의 최신 AMCL pose와 odom pose를 실제 시험 시작값으로 저장한다.
4. `/robot1/cmd_vel`에 `linear.x=0.05m/s`, `angular.z=0`만 10초간 발행한다.
5. 0 속도를 발행하고 2초간 기다린 뒤 실제 종료 위치를 계산한다. 시험 중
   AMCL이 갱신됐으면 AMCL을 사용하고, 아니면 시작 AMCL에 odom 변화량을
   투영한 위치를 사용한다.
6. 시작 방향으로 정확히 0.5m 이동한 예상 위치와 실제 위치의 오차를 계산한다.

명목 시작점은 시작 조건을 검사하기 위한 기준이고, 이동 오차 계산의 기준은
Nav2가 정확히 명목 좌표에 멈췄다고 가정하지 않고 **3번에서 측정한 실제 시작
AMCL pose**를 사용한다. 시작 pose가 `(x0, y0, yaw0)`이면 예상 종료 위치는
`(x0 + 0.5*cos(yaw0), y0 + 0.5*sin(yaw0))`이다. 현재 공용
`maps/map.yaml`에서 `(0.80, 0.20)`부터 `(0.80, 0.70)`까지는 free cell이며,
지도상 가장 가까운 벽과 약 0.76m 이상 떨어져 있다. 그래도 이 시험은 실시간
장애물 정지를 일부러 넣지 않았으므로 실행 직전 실제 전방 0.5m 공간을 직접
확인해야 한다.

결과는 `cmd_vel_distance_test/result`에 JSON으로 한 번 발행되고 latched된다.
주요 값은 다음과 같다.

| 결과 필드 | 의미 |
|---|---|
| `actual_pose_source` | 실제 위치 계산에 사용한 `amcl` 또는 `odom_projected` |
| `expected_amcl`, `actual_pose` | 예상/실제 종료 map pose |
| `actual_forward_m` | 시작 방향으로 실제 전진한 거리 |
| `actual_lateral_m` | 시작 방향 기준 좌우 편차(좌측이 양수) |
| `forward_error_m` | `actual_forward_m - 0.5m` |
| `position_error_m` | 예상점과 실제점 사이의 2D 직선거리 |
| `yaw_error_deg` | 시작 방향 대비 종료 방향 변화 |
| `odom_distance_m` | 같은 구간의 odom상 이동거리(AMCL 결과와 비교용) |

현재 설정은 `config/cmd_vel_distance_test.yaml`에 있다. 시작 허용 오차는
위치 0.20m, 방향 15도다. 정지 중 AMCL은 새 pose를 계속 발행하지 않을 수
있으므로 AMCL freshness를 시작 조건으로 쓰지 않는다. 시험 중 안전 확인에는
계속 들어오는 odom만 사용한다. Nav2 cancel 서비스가 없으면 복수 `cmd_vel`
발행 가능성을 막기 위해 기다린다.

```bash
tools/test_cmd_vel_distance.sh 1
```

이 한 명령이 시험 노드 사전 실행, 시작점 Nav2 이동, 사용자 전방 확인, 직진,
결과 출력, 로그 저장을 순서대로 수행한다. 노드를 Nav2 목표보다 먼저 실행하는
이유는 `/amcl_pose`가 latched 토픽이 아니어서 정지 후 실행한 새 구독자는 직전
pose를 받지 못할 수 있기 때문이다. 시험 중 Ctrl+C를 누르면 0 Twist를 발행하고
종료한다.

ROS-free 오차 계산 단위테스트 5개와, 가짜 AMCL/odom을 `cmd_vel`에 맞춰 이동시킨
노드 통합 시험에서 `WAITING → STARTING → MOVING → SETTLING → COMPLETE` 및
결과 JSON 발행까지 확인했다. 실제 robot1의 0.5m 이동 오차는 아직 측정 전이다.

## cmd_vel_route_follower (검증된 route 재생 도구)

나중에 정상 Nav2 주행에서 기록한 dense global path를 route YAML로 받아,
런타임에는 Nav2/AMCL/TF 없이 시작점에 odom delta만 합성해 `cmd_vel`로 추종한다.
35도 이상 꺾이는 구간은 lookahead를 코너점에서 차단하고, 코너 0.35m 전부터
감속한 뒤 코너 0.06m 안에서 정지한다. 다음 구간 방향으로 4도 이내 제자리
회전이 끝나야 직진을 재개하므로 벽 코너 안쪽을 대각선으로 자르지 않는다.

실행 파일, 파라미터, route template을 제공한다. route 파일은 검증 완료 뒤
`ready: true`로 명시해야 실행되도록 방어해, 임시 좌표를 실제 로봇에 보내는 것을
막는다. 상세 입력 절차는 `docs/cmd-vel-route-replay.md`에 있다.

```bash
ros2 run sensor_recovery cmd_vel_route_follower --ros-args \
  -r __ns:=/robot1 \
  --params-file src/sensor_recovery/config/cmd_vel_route.yaml \
  -p route_file:=/absolute/path/to/robot1_undock_to_goal.yaml
```

## 수동 Nav2 → cmd_vel takeover 시험

LiDAR를 실제로 끄기 전에 Nav2 주행 중간 제어권 전환만 따로 검증한다.
`tools/test_nav2_cmd_vel_takeover.sh`가 fallback controller를 먼저 실행해 현재
Nav2 `/plan`, AMCL, odom, compressed Depth와 static map을 캐시한다. RViz2에서
Nav2 Goal을 지정해 주행시킨 뒤 터미널에서 Enter를 누르면 다음 순서로 동작한다.

1. 현재 pose, Nav2 goal, 현재 위치 이후의 남은 `/plan`을 저장한다.
2. Nav2 goal 취소를 요청하고 action 상태에서 실제 취소 완료까지 0속도를 유지한다.
3. 취소가 확인되면 1초간 정지한 뒤 저장된 남은 경로를 odom+Depth 기반
   `cmd_vel`로 추종한다.
4. takeover 도중 새 Nav2 goal이 들어오면 다시 정지·취소해 두 제어기가 동시에
   `cmd_vel_nav`를 발행하지 않게 한다.

2026-08-08 robot1 실측에서는 약 4.35m 주행 후 odom 기반 추정과 AMCL 사이에
약 0.27m 횡방향 차이가 발생했다. 목표 좌표 자체에 고정 offset을 더하지 않고,
robot1의 fault 이후 상대 odom 변위에 아래 보정을 점진적으로 적용한다.

- 이동 거리 scale: `0.986`
- 상대 이동 방향 보정: `+4.0deg`
- 상대 yaw 변화 scale: `0.92`

이 값은 robot1에만 적용하며 robot2는 별도 실측 전까지 무보정이다. Depth가
장애물/노이즈를 감지해 정지한 경우에는 clear 판정이 연속 0.5초 유지된 뒤
다시 출발한다.

이 시험은 dock/undock과 LiDAR on/off를 수행하지 않는다. 터미널 1에서
`mapnav 1`, 터미널 2에서 아래 명령만 실행한다.

```bash
tools/test_nav2_cmd_vel_takeover.sh 1
```

## 실제 LiDAR OFF → fallback 도착 → LiDAR ON 시험

실제 운용에서는 `mapnav 1`만 실행해도 map/AMCL/Nav2와 함께 LiDAR watchdog,
fallback controller가 기본으로 실행된다. LiDAR 기능을 의도적으로 빼야 할 때만
`mapnav 1 false`를 사용한다.

LiDAR 고장 시험은 두 번째 터미널에서 아래 스크립트 하나만 실행한다. 이
스크립트는 Nav2나 fallback 노드를 중복 실행하지 않고, Enter 입력에 맞춰 실제
LiDAR 드라이버를 OFF/ON만 한다.

```bash
tools/test_lidar_fault_cycle.sh 1
```

시험 순서는 Nav2 주행 → LiDAR OFF → 5초 후 자동 fallback 전환 → cmd_vel 목적지
도착(`SUCCEEDED`) → LiDAR ON이다. 목적지에 이미 도착한 뒤 LiDAR가 복구되면
완료된 Nav2 goal은 다시 보내지 않는다. 로봇은 계속 정지하고 AMCL nomotion
update를 요청하며, fresh AMCL pose 3개가 연속으로 안정 범위 안에 들어오면
`RECOVERY_POSITION_CHECK`에 목표와 실제 위치 오차를 기록한다. 5초 안에 안정
위치를 얻지 못해도 Nav2를 억지로 재개하지 않고 정지한 채 계속 기다린다.

## fallback_route_test (이전 코드 보존, 지금 단계에서는 실행하지 않음)

LiDAR 장애 감지/depth 안전판정과 경로 추종을 한 번에 시험하지 않고, 먼저
fallback의 저속 `cmd_vel` 경로 추종만 실기로 검증하기 위한 도구다. Nav2는
로봇을 시작점에 배치하는 데만 사용하고, `fallback_test/start` 호출 이후에는
활성 Nav2 목표를 취소한 뒤 AMCL 시작 pose + odom 변화량으로만 위치를 추정한다.
LiDAR는 끄지 않으며 전방 비상정지에만 사용한다. 따라서 이 테스트가 성공해도
depth 기반 장애물 회피나 실제 LiDAR FAULT 전환이 검증된 것은 아니다.

공용 `maps/map.yaml`에서 footprint 반경 0.20m + 여유 0.05m를 적용해 다음 두
경로를 정했다. 노드는 실행 시 수신한 `/map`으로 모든 구간을 다시 검사하며,
안전 여유 검사를 통과하지 못하면 시작 서비스를 거부한다.

| 경로 | map 좌표 | 목적 |
|---|---|---|
| `straight` | `(0.80, 0.20) → (0.80, 2.80)` | 2.60m 직선 추종 |
| `wall_corner` | `(-2.40, 0.50) → (-1.45, 0.50) → (-1.45, 1.80)` | 내부 벽 아래를 지나 오른쪽 면을 따라 90도 회전 |

```bash
ros2 run sensor_recovery fallback_route_test --ros-args \
  -r __ns:=/robot1 \
  --params-file src/sensor_recovery/config/fallback_route_test.yaml \
  -p route_name:=straight

ros2 service call /robot1/fallback_test/start std_srvs/srv/Trigger '{}'
ros2 service call /robot1/fallback_test/stop std_srvs/srv/Trigger '{}'
```

시작 조건은 `/map`, 최신 `/odom`, 최신 `/scan`, AMCL pose, 시작점 0.20m 이내다.
주행 속도/가감속은 실제 fallback과 동일하게 0.05m/s, 0.20rad/s,
0.15m/s², 0.50rad/s²로 제한한다. `/scan`이 1초 이상 stale하거나 전방 ±35도
안에 0.35m보다 가까운 물체가 있으면 0 속도를 발행한다. 상태와 RViz용 경로는
각각 `fallback_test/state`, `fallback_test/path`로 확인할 수 있다.

## lidar_replacement_request

`lidar_fallback_controller`처럼 직접 주행하지 않는다. LiDAR가 죽으면 그냥
멈추고, "이 로봇 대신 다른 로봇이 가야 한다"는 신호만 발행한다 — 실제로
어느 로봇을 보낼지는 사람 또는 이후 Mission Manager가 판단한다.

- `* → FAULT`: `/cmd_vel`에 0 Twist 발행, `navigate_to_pose`의 활성 goal
  전부 취소, 그 순간의 `/plan` 마지막 지점(원래 목적지)을 저장.
- `→ ALIVE`: `replacement_needed`를 false로 되돌리고, **기본적으로는 거기서
  끝**(자동으로 목적지를 재전송하지 않음) — Mission Manager가 그 사이 다른
  로봇으로 재할당했을 수 있어서, 이 로봇이 임의로 예전 목적지를 향해
  재출발하면 두 로봇이 같은 곳으로 동시에 갈 수 있다. 실제로 뭘 할지는
  Mission Manager나 운영자가 `replacement_needed`/`pending_goal`을 보고
  명령해야 한다.

### Parameters

| 이름 | 기본값 | 설명 |
|---|---|---|
| `navigate_action` | `navigate_to_pose` | 사용할 Nav2 액션 이름 |
| `auto_resume_on_recovery` | `false` | `true`면 ALIVE 시 저장해둔 목적지로 자동 재출발 (Mission Manager 없이 단독 테스트할 때만 켠다) |

### Topics

- 구독: `lidar_state`(String), `plan`(nav_msgs/Path)
- 발행: `cmd_vel`(Twist, FAULT 시 0 한 번), `replacement_needed`
  (`std_msgs/Bool`, `TRANSIENT_LOCAL` — 늦게 구독해도 현재 상태 바로 받음,
  구독자도 `--qos-durability transient_local` 필요), `pending_goal`
  (`geometry_msgs/PoseStamped`, FAULT 시 목적지 1회 발행)
- 액션 클라이언트: `navigate_to_pose` (취소는 항상 수행. 재개는
  `auto_resume_on_recovery:=true`일 때만)

### Run

```bash
ros2 run sensor_recovery lidar_replacement_request --ros-args -r __ns:=/robot1
# 단독 테스트로 자동 재개까지 보고 싶으면:
ros2 run sensor_recovery lidar_replacement_request --ros-args \
  -r __ns:=/robot1 -p auto_resume_on_recovery:=true
```

### 검증 상태

단일 프로세스 통합 테스트로 두 모드 다 확인했다. 기본값(`false`)에서는 ALIVE 시
`replacement_needed=False`만 발행하고 재개 시도 자체를 안 함(Mission
Manager 대기 로그만 찍음), `auto_resume_on_recovery:=true`에서는 기존처럼
저장된 목적지로 재개를 시도함(Nav2 액션 서버 없을 때 에러 로그만 남고
크래시 없음). FAULT 시 `replacement_needed=True` + 목적지 발행 + cmd_vel
0 발행도 두 모드 공통으로 확인했다. 실기 기본 경로는 watchdog과
`lidar_fallback_controller`를 함께 사용하는 fault-cycle 시험이다.

**`lidar_fallback_controller`와 동시에 같은 로봇에서 실행하지 않는다** —
둘 다 FAULT 시 Nav2 goal을 취소하고 `cmd_vel`을 발행하려고 해서 충돌한다.
