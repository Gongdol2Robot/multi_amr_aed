# Robot2 Nav2 주행 정지 문제 해결 기록

## 결과

2026-08-07 robot2 실기 시험에서 기존에 반복 정지하던 경로를 정상 완주했다.

- 1차 목표: `(0.08, 0.06) → (2.77, 0.42)` 성공
- 기존 실패 목표: `(2.76, 0.37) → (2.33, 2.38)` 약 30.4초 만에 성공
- 반대 방향 복귀: `(2.19, 2.34) → (2.77, 0.42)` 약 28.0초 만에 성공
- 세 주행 모두 `Failed to make progress` 없음
- 검증 로그: `logs/mapnav_robot2_20260807_135056.log`

## 증상

첫 번째 직선 목표는 성공하지만, 지도 중앙의 긴 가로벽을 왼쪽으로 돌아가야
하는 두 번째 목표에서는 robot2가 `(2.32, 0.70)` 부근에서 정지했다. 이후
Nav2가 약 20초마다 `Failed to make progress`를 발생시키고 costmap clear와
재계획을 반복했지만 같은 위치를 벗어나지 못했다.

## 진단 과정

### 1. Progress checker 보강

기존 `SimpleProgressChecker`는 제자리 회전을 진행으로 보지 않는다. 이를 위치와
회전을 모두 검사하는 `nav2_controller::PoseProgressChecker`로 교체했다.

- `required_movement_angle: 0.35 rad`
- `required_movement_radius: 0.10 m`
- `movement_time_allowance: 20.0 s`

회전 중의 오판정과 저속 이동의 오판정은 줄었지만, 문제 경로의 정지 자체는
해결되지 않았다. 따라서 progress timeout만 늘리는 방식은 사용하지 않았다.

### 2. `NAV_TRACE` 추가

`turtlebot4_map_navigation/nav_diagnostics.py`를 추가해 주행 중 다음 값을 1초마다
같은 로그 줄에 기록했다.

- `cmd_vel_nav`: 로컬 컨트롤러가 선택한 원본 속도
- `cmd_vel`: velocity smoother를 통과한 실제 출력 속도
- odom/AMCL 위치와 방향
- 1초간 odom 이동 거리
- 전역 경로 pose 수와 시작/종료 좌표

이를 통해 로컬 컨트롤러, smoother, 구동부 중 어느 단계에서 정지가 시작되는지
구분했다.

### 3. 직접 원인 확인

DWB 사용 로그 `mapnav_robot2_20260807_125925.log`에서 문제 위치 진입 직후
다음 순서가 확인됐다.

1. DWB 회전 명령이 `+0.7 → +1.0 → -0.5 → -0.7 → +0.4 → +0.2 rad/s`로
   방향을 반복해서 바꿨다.
2. 이후 전역 경로가 125~126 pose로 계속 존재하는데도
   `cmd_vel_nav=(0.0, 0.0)`을 반복 출력했다.
3. smoother 출력도 `(0.0, 0.0)`이고 odom 이동량도 `0.000 m`였다.
4. 같은 실행에서 다른 방향의 목표는 정상 속도로 성공했다.

따라서 Wi-Fi, robot2 베이스, velocity smoother, TF, 전역 경로 생성 문제가
아니라 DWB가 벽 우회 코너에서 정지 trajectory에 고착된 것이 직접 원인이었다.

## 최종 변경

### DWB를 Regulated Pure Pursuit로 교체

`nav2_aed.yaml`의 `FollowPath`를 다음 플러그인으로 교체했다.

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.20
  lookahead_dist: 0.35
  min_lookahead_dist: 0.25
  max_lookahead_dist: 0.55
  lookahead_time: 1.5
  use_velocity_scaled_lookahead_dist: true
  use_rotate_to_heading: true
  rotate_to_heading_min_angle: 0.60
  rotate_to_heading_angular_vel: 0.60
  allow_reversing: false
  use_collision_detection: true
  max_allowed_time_to_collision_up_to_carrot: 1.0
```

RPP는 경로 방향과 로봇 방향의 차이가 0.60 rad보다 크면 먼저 한 방향으로
제자리 정렬한 뒤 경로의 lookahead point를 추종한다. DWB처럼 여러 trajectory의
점수를 비교하다 정지 후보를 선택하는 구조가 아니므로 이번 코너에서 발생한
회전 방향 진동과 `(0, 0)` 고착을 피했다.

충돌 감지와 1초 전방 충돌 예측은 계속 활성화했다. 즉 장애물 검사를 끄거나
로봇 반경을 줄여 통과시킨 해결책이 아니다.

### 시작 안정화와 진단 유지

- localization initializer가 `odom → base_link → rplidar_link` TF를 확인한
  다음 초기 pose와 Nav2 시작을 진행한다.
- `nav_diagnostics`는 이후 회귀 분석을 위해 계속 실행한다.
- `mapnav`는 화면 로그를 `logs/mapnav_robotN_*.log`에도 저장한다.
- `nav2_regulated_pure_pursuit_controller`를 런타임 의존성으로 명시했다.

## 검증

- `controller_server` lifecycle configure smoke test에서 RPP 플러그인 로드 성공
- `aed_bringup`, `turtlebot4_map_navigation` 빌드 성공
- map navigation 패키지 테스트: `5 passed, 1 skipped`
- robot2 실기에서 문제 경로와 반대 방향 경로 모두 성공

## 재현 절차

LiDAR를 켜고 fallback/watchdog/mission manager는 실행하지 않은 일반 Nav2
상태에서 시험한다.

### 터미널 1 — robot2 Nav2 실행

```bash
cd /home/rokey/git/multi_amr_aed_jaehyeon
source tools/aliases.sh
aedenv
pf 2
mapnav 2
```

### 터미널 2 — 목표 전송

```bash
cd /home/rokey/git/multi_amr_aed_jaehyeon
aedenv

navgoal() {
  ros2 action send_goal \
    /robot2/navigate_to_pose \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
    --feedback
}

navgoal 2.77 0.42
navgoal 2.33 2.38
navgoal 2.77 0.42
```

### 터미널 3 — 핵심 로그 확인

```bash
cd /home/rokey/git/multi_amr_aed_jaehyeon

tail -f "$(ls -1t logs/mapnav_robot2_*.log | head -n 1)" \
  | grep --line-buffered -E \
  'RegulatedPurePursuit|NAV_TRACE|Failed to make progress|Goal succeeded|Goal failed'
```
