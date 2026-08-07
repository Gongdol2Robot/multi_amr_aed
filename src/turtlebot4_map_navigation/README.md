# turtlebot4_map_navigation

저장된 지도를 노트북의 RViz에 표시하고, AMCL로 TurtleBot 4의 위치를
추정한 뒤 RViz에서 선택한 목적지까지 Nav2로 주행하는 패키지입니다.

## 빌드

```bash
cd <rokey_ws>
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-up-to \
  turtlebot4_map_navigation
source install/setup.bash
```

## 실행

SLAM, teleop, RC카 추종 노드를 모두 종료한 뒤 실행합니다.

```bash
ros2 launch turtlebot4_map_navigation map_navigation.launch.py
```

기본 지도는 두 로봇의 공용 좌표계 지도입니다.

```text
multi_amr_aed/maps/map.yaml
```

다른 지도를 사용하려면 워크스페이스를 기준으로 상대 경로를 전달합니다.

```bash
ros2 launch turtlebot4_map_navigation map_navigation.launch.py \
  map:=src/turtlebot4_mapping/maps/office_map.yaml
```

## RViz에서 주행

기본적으로 `dock_poses.yaml`에서 실행 namespace에 맞는 robot1 또는 robot2의
Dock 위치와 방향을 읽어 AMCL 초기 위치로 자동 입력합니다.

1. 터미널에 `Localization is ready; starting Nav2 now.`가 표시될 때까지
   기다립니다.
2. LiDAR와 지도 윤곽이 맞는지 확인합니다.
3. RViz의 `Nav2 Goal`을 선택합니다.
4. 이동할 위치를 클릭하고 최종 방향으로 드래그합니다.

Nav2가 전역 경로와 지역 경로를 계산하고 `/robot1/cmd_vel`을 통해 로봇을
주행시킵니다. 새 목적지를 지정하면 진행 중인 목적지가 갱신됩니다.

Localization 준비 노드는 `map_server`, `amcl`, scan, odometry, 초기 자세,
`odom -> base_link -> LiDAR` 로봇 TF, `map -> base_link` TF를 모두 확인한
후에만 Nav2를 실행합니다. 센서 토픽이 먼저 보여도 로봇 TF가 아직 discovery에
들어오지 않았다면 초기 자세 계산과 Nav2 활성화를 시작하지 않습니다.

기본 동작은 로봇별 Dock 자세를 자동으로 사용하는 것입니다.

```bash
ros2 launch turtlebot4_map_navigation map_navigation.launch.py \
  namespace:=robot1 rviz:=true
```

Dock이 아닌 곳에서 시작한다면 자동 입력을 끄고 RViz의 `2D Pose Estimate`로
실제 위치와 방향을 지정합니다.

```bash
ros2 launch turtlebot4_map_navigation map_navigation.launch.py \
  namespace:=robot1 auto_initial_pose:=false
```

## 주행 progress 판정

`nav2_aed.yaml`은 위치 이동만 검사하는 `SimpleProgressChecker` 대신 위치와
회전을 함께 검사하는 `PoseProgressChecker`를 사용합니다. 경로를 향해 제자리
회전하는 동안 20초가 지났다는 이유만으로 `Failed to make progress`가 발생하지
않게 하기 위한 설정입니다.

로컬 경로 추종기는 DWB 대신 Regulated Pure Pursuit를 사용합니다. 실기
`NAV_TRACE`에서 DWB가 벽 우회 코너에서 회전 방향을 반복해서 바꾼 뒤 전역
경로가 있는데도 `(0, 0)` 명령을 계속 선택한 것이 확인됐기 때문입니다. RPP는
경로와의 각도 차이가 0.60 rad보다 크면 먼저 제자리 정렬한 뒤 최대 0.20 m/s로
경로를 추종합니다. 전방 충돌 예측은 활성 상태입니다.

실기에서 벽을 우회하던 robot2가 76초 동안 약 0.50 m 이동했는데도 20초마다
0.25 m를 채우지 못해 recovery가 반복된 로그를 반영해, 이동 진행 기준은
0.10 m로 조정했습니다. 시간 제한은 20초로 유지해 실제 정지를 무한정
허용하지 않습니다.

`map_navigation.launch.py`는 주행 중 1초마다 `NAV_TRACE`도 남깁니다.
`cmd_vel_nav`(로컬 컨트롤러 출력), `cmd_vel`(velocity smoother 출력), odom/AMCL 자세,
최근 전역 경로를 같은 줄에서 비교할 수 있어 다음 실행 로그만으로도
컨트롤러 문제와 베이스 구동 문제를 구분할 수 있습니다.

실기에서는 다음 두 현상을 구분해야 합니다.

- 제자리 회전 중 progress abort가 사라지고 이후 전진하면 이번 설정의 대상 문제
- pose 자체가 변하지 않거나 `Transform data too old`가 동반되면 구동부 또는
  scan/TF 통신 문제이므로 progress timeout을 더 늘려서 숨기면 안 됨

## 안전 사항

- 처음에는 로봇 가까이에서 비상 정지를 준비합니다.
- 지도와 실제 장애물 배치가 달라졌는지 확인합니다.
- 계단, 유리, 낮은 장애물은 2D LiDAR가 감지하지 못할 수 있습니다.
- 다른 노드가 `/robot1/cmd_vel`을 동시에 발행하지 않게 합니다.
- 자동 초기 위치를 사용할 때는 로봇의 실제 시작 위치와 방향이 맵 저장
  자세와 일치하는지 확인합니다.
