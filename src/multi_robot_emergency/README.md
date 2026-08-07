# multi_robot_emergency

각 로봇 PC에서 기존 단독 Nav2/RViz를 실행하고, 중앙 PC에서는 두 Nav2
Planner가 계산한 실제 경로 길이를 비교해 짧은 로봇만 출동시키는 패키지입니다.

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

목표는 `map` 좌표의 `PoseStamped`로 보냅니다.

```bash
ros2 topic pub --once /emergency/request geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.2, y: 2.4}, orientation: {w: 1.0}}}"
```

또는 RViz 툴바에서 **Publish Point**를 선택하고 지도 좌표를 클릭합니다.
`/clicked_point`, `/robot1/clicked_point`, `/robot2/clicked_point`를 모두
구독하므로 중앙 노드는 계속 켜둔 상태에서 어느 RViz에서든 클릭할 때마다
두 로봇의 경로를 다시 계산합니다. `Nav2 Goal`은 사용하지 않습니다.

중앙 노드는 다음 순서로 처리합니다.

1. `/robot1/amcl_pose`, `/robot2/amcl_pose` 최신 상태 확인
2. 두 `/<robot>/compute_path_to_pose` Action에 같은 목표 동시 요청
3. 반환된 `nav_msgs/Path`의 모든 구간 길이와 회전 비용 계산
4. Camera2가 판정한 혼잡 상태와 골목 통과 길이를 ETA에 반영
5. 경로 실패/시간 초과 로봇 제외
6. 최종 ETA가 가장 짧은 로봇에만 임무 전송

결과 확인:

```bash
ros2 topic echo /emergency/status
ros2 topic echo /emergency/selected_robot
ros2 topic echo /emergency/path_distance/robot1
ros2 topic echo /emergency/path_distance/robot2
ros2 topic echo /emergency/eta/predicted/robot1
ros2 topic echo /emergency/eta/predicted/robot2
ros2 topic echo /emergency/eta/actual/robot1
ros2 topic echo /emergency/eta/actual/robot2
ros2 topic echo /emergency/eta/result
```

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
구역을 지나는 후보 경로를 제외합니다. `CLEAR` 또는 `UNKNOWN`이면
`crowd_delay`는 0입니다. RViz 표시 토픽과 확인용 토픽은 다음과 같습니다.

```bash
ros2 topic echo /camera_alley/vision/crowd_level
ros2 topic echo /camera_alley/vision/person_count
ros2 topic echo /emergency/crowd/state
ros2 topic echo /emergency/crowd_markers
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
