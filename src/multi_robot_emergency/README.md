# multi_robot_emergency

각 로봇 PC에서 기존 단독 Nav2/RViz를 실행하고, 중앙 PC에서는 두 Nav2
Planner의 ETA를 비교해 기본적으로 가장 빠른 로봇을 출동시키는 패키지입니다.
목표시간 미달 위험이 크면 두 로봇을 동시에 출동시키며, 먼저 도착한 로봇이
생기면 늦은 로봇의 목표를 취소하고 출발 위치로 복귀시킵니다.

중앙 런치는 Nav2, RViz, TF bridge를 실행하지 않습니다.

## Build

```bash
cd ~/turtlebot4_ws/multi_amr_aed
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install \
  --packages-select multi_robot_emergency
source install/setup.bash
```

## Robot 1 PC

각 명령은 별도 터미널에서 실행합니다.

```bash
source ~/turtlebot4_ws/multi_amr_aed/tools/aliases.sh
aedenv
loc 1
nav 1
rv 1
```

## Robot 2 PC

Robot 1과 동일한 공용 `multi_amr_aed/maps/map.yaml`을 사용합니다.

```bash
source ~/turtlebot4_ws/multi_amr_aed/tools/aliases.sh
aedenv
loc 2
nav 2
rv 2
```

## Central PC

먼저 두 Planner가 모두 보이는지 확인합니다.

```bash
source /etc/turtlebot4_discovery/setup.bash
export ROS_SUPER_CLIENT=True
source ~/turtlebot4_ws/multi_amr_aed/install/setup.bash

ros2 action info /robot1/compute_path_to_pose
ros2 action info /robot2/compute_path_to_pose
```

선정 로그만 시험하려면:

```bash
ros2 launch multi_robot_emergency central_dispatch.launch.py
```

선택된 로봇을 실제 출동시키려면:

```bash
ros2 launch multi_robot_emergency central_dispatch.launch.py \
  dispatch_enabled:=true
```

기본 동시출발 기준은 목표시간 30초의 85%인 25.5초입니다. 가장 빠른
후보의 ETA도 25.5초 이상이고 두 로봇 모두 유효한 경로가 있을 때 두 대를
출동시킵니다. 기준은 실행할 때 조정할 수 있습니다.

```bash
ros2 launch multi_robot_emergency central_dispatch.launch.py \
  dispatch_enabled:=true \
  dual_dispatch_enabled:=true \
  target_arrival_time_sec:=30.0 \
  dual_dispatch_trigger_ratio:=0.85
```

단축 명령은 `central <실주행> <목표시간초> <발동비율> <동시출발>` 순서입니다.
기본 설정으로 실주행할 때는 `central true`, 목표시간을 40초로 바꾸려면
`central true 40 0.85 true`를 사용합니다.

목표는 `map` 좌표의 `PoseStamped`로 보냅니다.

```bash
ros2 topic pub --once /emergency/request geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.2, y: 2.4}, orientation: {w: 1.0}}}"
```

요청 좌표는 로봇이 그대로 밟고 지나갈 Nav2 Goal이 아니라 **환자의 위치**로
취급합니다. 기본 설정에서는 각 로봇의 현재 위치와 환자 위치를 잇는 방향을
기준으로 환자에게서 0.15m 떨어진 지점을 Nav2 Goal로 만들고, 최종 자세는
환자를 바라보도록 설정합니다. 따라서 경로 거리와 ETA도 실제 0.15m 정지
지점까지 계산됩니다. 각 로봇의 정지점은 현재 위치에서 환자를 향하는 접근
방향에만 만들며, 후보 경로 계산 중 정지점을 옆이나 반대편으로 강제 이동하지
않습니다.

```bash
ros2 launch multi_robot_emergency central_dispatch.launch.py \
  patient_standoff_enabled:=true \
  patient_standoff_distance_m:=0.15 \
  dual_robot_proximity_threshold_m:=0.40 \
  dual_robot_proximity_confirm_sec:=0.50 \
  dual_robot_proximity_grace_sec:=2.0
```

두 대가 동시 출동한 경우 출발 직후 2초는 감시에서 제외합니다. 이후 두 로봇
중심이 0.40m 이하인 상태가 0.50초 이상 계속되면, 환자까지 직선거리가 더 먼
로봇의 현재 목표를 취소하고 저장된 출발 위치로 복귀시킵니다. 환자와의 거리가
같으면 ETA 1순위 로봇을 남기고 다른 로봇을 복귀시킵니다.

검출 노드가 이미 안전 정지 좌표를 발행하는 경우에만
`patient_standoff_enabled:=false`로 끕니다.

조력자 탐색 노드가 `HELPER_ARRIVED`를 발행하면 도착 로봇은 출동 직전에
저장한 위치로 자동 복귀합니다. 기본값은
`return_after_helper_enabled:=true`이며, 현장에 남겨야 하는 시험에서는
`false`로 끌 수 있습니다. 조력자 확인 TTS와 5초 인계 대기는
`helper_mission`이 담당합니다.

`central_dispatch.launch.py`는 기본적으로 두 로봇의 `aed_vision`과
`helper_mission`도 함께 실행합니다. HMI 자체는 기존 정책대로 별도
프로세스입니다. 로봇 Vision은 OAK-D raw preview를 각각 한 번만 구독하고,
HMI는 `/robotN/vision/debug/compressed`를 받아 중복 raw 전송을 피합니다.
개별 디버깅 시에는 `start_robot_vision:=false` 또는
`start_helper_mission:=false`로 제외할 수 있습니다.

또는 RViz 툴바에서 **Publish Point**를 선택하고 지도 좌표를 클릭합니다.
`/clicked_point`, `/robot1/clicked_point`, `/robot2/clicked_point`를 모두
구독하므로 중앙 노드는 계속 켜둔 상태에서 어느 RViz에서든 클릭할 때마다
두 로봇의 경로를 다시 계산합니다. `Nav2 Goal`은 사용하지 않습니다.

중앙 노드는 다음 순서로 처리합니다.

1. `/robot1/amcl_pose`, `/robot2/amcl_pose` 최신 상태 확인
2. 환자 앞 0.15m에 로봇별 정지 목표를 만들고 두
   `/<robot>/compute_path_to_pose` Action에 동시 요청
3. 반환된 `nav_msgs/Path`의 모든 구간 길이와 회전 비용 계산
4. Camera2가 판정한 혼잡 상태와 골목 통과 길이를 ETA에 반영
5. 경로 실패/시간 초과 로봇 제외
6. 최단 ETA가 목표시간의 85% 미만이면 가장 빠른 로봇에만 임무 전송
7. 최단 ETA가 기준 이상이면 유효한 두 로봇에 동시 임무 전송
8. 먼저 도착한 로봇이 생기면 늦은 로봇의 목표를 취소하고 저장한 출발
   Pose로 `ROLE_RETURN` 임무 전송

YOLO 연동 시 `/camera_open/vision/emergency_event`와
`/camera_alley/vision/emergency_event`의 `CONFIRMED` 전이만 새 요청으로
받습니다. 프레임마다 나오는 `fallen_location`은 출동 트리거로 사용하지 않아
같은 검출이 반복 출동으로 이어지지 않습니다. 이벤트의 map 좌표는 위의
0.15m 환자 정지 처리에 그대로 들어갑니다.

혼잡도는 비전이 발행하는 문자열 `0/1/2/3`을 각각
`CLEAR/BUSY/CROWDED/BLOCKED`로 변환합니다. 시간 보정은 비전 JSON의
`crowd_time_multiplier`를 사용하지 않고, 이 패키지의 AMR 실측 속도
`0.20/0.15/0.10m/s`와 3단계 통행 불가 정책을 한 번만 적용합니다.

복귀 전환 시 executor는 기존 Nav2 Goal의 취소 응답을 기다리고 0.5초 뒤
복귀 Goal을 보냅니다. 취소 경합 등으로 복귀 Goal이 `ABORTED` 또는
`CANCELED`되면 기본 15초 동안 0.5초 간격으로 다시 시도합니다. 이 제한을
넘겨도 경로가 생성되지 않으면 `/emergency/status`에
`NAVIGATION_ERROR`가 남으므로 물리적 장애물과 costmap을 확인해야 합니다.

결과 확인:

```bash
ros2 topic echo /emergency/status
ros2 topic echo /emergency/selected_robot
ros2 topic echo /emergency/dispatched_robots
ros2 topic echo /emergency/path_distance/robot1
ros2 topic echo /emergency/path_distance/robot2
ros2 topic echo /emergency/eta/predicted/robot1
ros2 topic echo /emergency/eta/predicted/robot2
ros2 topic echo /emergency/eta/actual/robot1
ros2 topic echo /emergency/eta/actual/robot2
ros2 topic echo /emergency/eta/result
```

`/emergency/selected_robot`은 현재 최우선 로봇 또는 먼저 도착한 로봇을
표시합니다. `/emergency/dispatched_robots`는 동시출발 여부, 출동·주행·복귀
중인 로봇 목록을 JSON 문자열로 발행합니다.

경로를 계산할 수 없는 로봇의 거리에는 `nan`이 발행됩니다.
정지 중에는 AMCL pose가 반복 발행되지 않으므로 기본적으로 마지막 수신 위치를
계속 사용합니다. 엄격한 시간 제한이 필요할 때만 `allow_stale_pose:=false`를
지정합니다.

기본 `use_planner_start:=true`에서는 중앙 노드가 오래된 AMCL 시작점을 강제로
넣지 않고 각 Nav2 Planner가 TF에서 현재 로봇 위치를 직접 사용합니다.
단, `dock_status.is_docked=true`이면 Dock 벽의 인플레이션에 시작점이 갇히지
않도록 실제 언도킹 방향으로 0.35m 이동한 예상 위치에서 경로를 계산합니다.
이 값은 `docked_start_offset_m`으로 조정할 수 있습니다.

후보 실제 경로도 계속 발행합니다.

- `/emergency/candidate_path/robot1`
- `/emergency/candidate_path/robot2`

## Camera2 혼잡도 연동

중앙 노드는 사람 수로 혼잡 여부를 다시 판정하지 않습니다. 비전 팀이 발행한
`/camera_alley/vision/crowd_level`(`std_msgs/msg/String`)을 최종 판정으로
구독합니다. `/camera_alley/vision/person_count`(`std_msgs/msg/UInt32`)은
로그와 상태 확인에만 사용합니다.

임시 단계는 `CLEAR`, `BUSY`, `CROWDED`, `BLOCKED`입니다. 중앙은 문자열의
목록 순서를 혼잡 단계로 사용하며 사람 수로 단계를 다시 판정하지 않습니다.
팀원이 단계명을 변경하면 `config/crowd_zones.yaml`의
`crowd_level_names`만 실제 발행 문자열과 맞추면 됩니다.

- 더 높은 단계가 0.5초 유지되면 중앙 단계를 상향
- 더 낮은 단계가 1.5초 유지되면 중앙 단계를 하향
- crowd level이 2초 이상 끊기면 `UNKNOWN`으로 처리하고 페널티 미적용
- 한 긴급 요청의 두 후보는 요청 시점에 고정한 동일 상태로 비교

혼잡 구역은 같은 설정 파일의 `crowd_zone_polygon`에 공용 `map` 좌표의
`x, y` 쌍으로 저장합니다. 최종 ETA는 다음과 같습니다.

```text
base_eta = 거리 / 0.20 + 회전각 / 0.70 + 큰 코너 수 * 4.0
crowd_delay = 골목 내부 거리 * (1 / 단계속도 - 1 / 0.20)
final_eta = base_eta + crowd_delay
```

단계별 기본 속도는 `0.20, 0.15, 0.10m/s`이며 `BLOCKED` 단계에서는 해당
polygon을 Nav2 keepout mask로 전역·지역 costmap에 넣습니다. 두 Planner는
mask 반영을 위해 1.2초 기다린 뒤 막힌 구역을 우회하는 경로를 각각 다시
생성합니다. 우회로 자체가 없을 때만 해당 후보의 경로 계산이 실패합니다.
`CLEAR` 또는 `UNKNOWN`이면 mask와 `crowd_delay`를 모두 해제합니다. RViz
표시 토픽과 확인용 토픽은 다음과 같습니다.

```bash
ros2 topic echo /camera_alley/vision/crowd_level
ros2 topic echo /camera_alley/vision/person_count
ros2 topic echo /emergency/crowd/state
ros2 topic echo /emergency/crowd_markers
ros2 topic echo /emergency/crowd_filter_info
ros2 topic echo /emergency/crowd_keepout_mask
ros2 topic echo /emergency/crowded_path_distance/robot1
ros2 topic echo /emergency/crowd_delay/robot1
```

## DB 연동용 ETA 토픽

중앙 노드는 DB나 대시보드에서 사용할 수 있도록 ETA를 초 단위로
발행합니다.

| 토픽 | 타입 | 발행 시점 | 값 |
|---|---|---|---|
| `/emergency/eta/predicted/robot1` | `std_msgs/msg/Float32` | Robot1 경로 계산 완료 | Robot1 예상 주행시간(초) |
| `/emergency/eta/predicted/robot2` | `std_msgs/msg/Float32` | Robot2 경로 계산 완료 | Robot2 예상 주행시간(초) |
| `/emergency/eta/actual/robot1` | `std_msgs/msg/Float32` | Robot1 정상 도착 | Robot1 실제 주행시간(초) |
| `/emergency/eta/actual/robot2` | `std_msgs/msg/Float32` | Robot2 정상 도착 | Robot2 실제 주행시간(초) |
| `/emergency/eta/result` | `std_msgs/msg/String` | 선택 로봇 정상 도착 | 요청별 최종 JSON 레코드 |

새 요청이 시작되면 로봇별 숫자 토픽은 `nan`으로 초기화됩니다. DB 저장은
요청 ID가 포함된 `/emergency/eta/result`를 기준으로 하는 것이 안전합니다.

```json
{"actual_arrival_sec":23.5,"error_sec":0.67,"predicted_eta_sec":22.83,"request_id":"emergency-002","robot_id":"robot2","stamp_sec":1786011309.024,"status":"ARRIVED"}
```
