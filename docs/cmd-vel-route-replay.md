# Nav2 기준 경로의 cmd_vel 단독 재현

## 목표

정상 LiDAR 상태에서 Nav2가 `undock 위치 → 목표 위치`로 만든 경로를 한 번
기록한다. 이후 실행에서는 Nav2 planner/controller를 사용하지 않고, 저장된
경로와 odom 피드백만으로 `/robot1/cmd_vel`을 발행해 같은 통로를 주행한다.
현재는 목표 좌표와 실제 기준 경로만 비어 있고 제어기와 기록 형식은 구현돼 있다.

## 경로를 얻는 방법

`mapnav`의 `nav_diagnostics`는 목표가 새로 들어오면 다음 두 레코드를 로그에
남긴다.

- `NAV_PATH_SNAPSHOT`: Nav2 global planner의 첫 전체 경로
- `NAV_EXECUTED_PATH`: 주행 중 약 3cm 간격으로 모은 실제 AMCL 궤적

나중에 받은 로그에서 두 경로를 비교한다. 기본 route는 벽 안전 여유가 포함된
planned path를 사용하고, 실제 주행이 지속적으로 한쪽으로 치우친 구간은 executed
path를 참고해 코너점과 보정 파라미터를 조정한다.

```bash
tools/extract_nav_route.py \
  logs/mapnav_robot1_YYYYMMDD_HHMMSS.log \
  src/sensor_recovery/config/robot1_undock_to_goal.yaml \
  --kind planned
```

2026-08-07 이전 형식의 로그는 planner 경로 첫 pose의 방향을 시작 yaw로
기록해 실제 방향 대신 `0°`가 들어갈 수 있다. 그런 로그는 같은 시점의
`NAV_TRACE amcl yaw`를 확인한 뒤 `--start-yaw-deg`로 명시한다. 새 로그는
`start_yaw_source: amcl_pose`와 함께 최신 AMCL 방향을 자동 기록한다.

## 꺾이는 구간 처리

일반 pure-pursuit만 사용하면 lookahead target이 코너 너머에 잡혀 로봇이 코너
안쪽을 대각선으로 자를 수 있다. 벽을 끼는 구간에서는 작은 위치 오차도 충돌로
이어지므로 다음 하이브리드 제어를 사용한다.

1. dense Nav2 path의 각 점에서 앞뒤 0.25m 구간 방향을 비교한다.
2. 방향 변화가 35도 이상인 점들을 hard-corner 후보로 잡는다.
3. 0.35m 안에 몰린 후보는 가장 큰 방향 변화 하나로 묶는다.
4. 코너 접근 중 lookahead index를 코너 index에서 강제로 자른다.
5. 코너 0.35m 전부터 선속도를 낮추고, 코너점 0.06m 안에서 완전히 정지한다.
6. 다음 구간 방향 오차가 4도 이내가 될 때까지 `linear.x=0`으로 제자리 회전한다.
7. 정렬 후 다음 직선 구간 추종을 시작한다.

따라서 코너에서는 `직선 주행 → 감속 → 정지 → 회전 → 직선 주행`이 명시적인
상태로 분리된다. 완만한 굴곡은 0.20m lookahead 제어로 연속 추종한다.

## 런타임 위치 추정

route 시작점과 시작 yaw를 map anchor로 두고, start 서비스 시점의 odom을
저장한다. 이후 위치와 방향은 odom delta를 anchor에 합성해 계산한다. 주행 중
AMCL, TF, Nav2 costmap은 사용하지 않는다. odom이 1초 이상 끊기거나 경로에서
0.30m 이상 이탈하면 0 Twist를 발행하고 `FAILED`가 된다.

## 정적 map 안전 검증

노드는 `/map`을 transient-local QoS로 받아 route를 실행하기 전에 직접 검사한다.
모든 dense path 구간이 known-free cell 안에 있고, 벽 clearance가 TurtleBot 반경
0.20m + 고정 여유 0.05m 이상이어야 한다. hard corner의 제자리 회전점도 같은
0.25m clearance 조건을 통과해야 한다. 하나라도 실패하거나 map이 아직 없으면
start 서비스를 거부한다. TurtleBot을 원형 footprint로 취급하므로 제자리 회전의
swept footprint도 같은 반경이다.

map은 주행 전 route 안전성 검증에 사용하고, 주행 중 위치 진행도는 계속 odom
delta로 계산한다. 정적 map에 없는 이동 장애물 처리는 이후 depth 안전 계층과
결합하는 단계에서 추가한다.

현재는 depth 안전 계층도 결합돼 있다. robot1에서 실제 수신되는
`oakd/stereo/image_raw/compressedDepth`를 16UC1로 복원하고, 중앙 및 회전
방향 ROI가 `CLEAR`가 아니면 즉시 0 속도와 `BLOCKED` 상태로 전환한다.
시작 서비스는 AMCL 시작 위치·방향, odom, 정적 map 검사, depth 상태가 모두
정상일 때만 승인한다. 속도 명령은 `cmd_vel_nav`로 보내 velocity smoother를
거친다.

정적 map clearance는 정확한 유클리드 거리 변환을 사용한다. 이전 8방향 BFS가
대각선 거리까지 한 셀로 계산해 실제 약 0.31m 여유를 0.20m로 과소평가하던
문제를 수정했으며, 현재 robot1 route 210개 구간이 모두 0.25m 기준을 통과한다.

## robot1 격리 시험

터미널 1에서 `mapnav 1`, 터미널 2에서 `tools/test_cmd_vel_route.sh 1`을
실행한다. dock/undock은 절대 자동 실행하지 않으며, 사용자가 로봇을 시작
위치에 직접 둔 뒤 실행한다. 스크립트는 follower 실행, 사전검사, start 호출,
결과 로그 저장만 담당한다. LiDAR는 끝까지 켜 둔다. 완료 시 `ROUTE_RESULT`에
goal 대비 AMCL 위치·방향 오차가 기록된다.

## 남은 검증

실제 route YAML 생성, 지도 clearance, 코너 검출, depth 수신까지 준비됐다.
이제 LiDAR를 켠 저속 cmd_vel 실기 결과로 코너 오차와 최종 위치 오차를
확인한 뒤 제어 파라미터를 조정하면 된다. 그 시험이 성공한 다음에만 실제
LiDAR FAULT 자동 전환 시험으로 넘어간다.
