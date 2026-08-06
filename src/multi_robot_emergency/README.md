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
3. 반환된 `nav_msgs/Path`의 모든 구간 길이 합산
4. 경로 실패/시간 초과 로봇 제외
5. 가장 짧은 로봇의 `/<robot>/navigate_to_pose`에만 목표 전송

결과 확인:

```bash
ros2 topic echo /emergency/status
ros2 topic echo /emergency/selected_robot
ros2 topic echo /emergency/path_distance/robot1
ros2 topic echo /emergency/path_distance/robot2
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
