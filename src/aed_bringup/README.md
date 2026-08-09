# aed_bringup

통합 담당: 이현민

멀티 TurtleBot4 시스템의 공통 Nav2 설정과 전체 기동 파일을 관리합니다.

## Nav2 configuration

`config/nav2_aed.yaml`은 기존 미니 프로젝트에서 실기 검증한 설정을
이관한 것입니다.

- 로봇 반경: `0.20 m`
- 최대 직진 속도: `0.20 m/s`
- 최대 탐색 회전 속도: `0.7 rad/s`
- 로컬·글로벌 inflation 반경: `0.22 m`
- DWB 장애물 critic 가중치: `1.50`
- 실제 TurtleBot4 운용을 위해 `use_sim_time: false`

두 로봇은 동일한 파일을 사용하고 namespace만 `/robot1`, `/robot2`로
분리합니다.

## Central dispatch

각 로봇 PC에서 Nav2를 따로 실행한 뒤 중앙 PC에서 경로비용 계산과 미션 배정을
실행합니다. 이 런치는 Nav2나 RViz를 실행하지 않습니다.

```bash
ros2 launch aed_bringup central_dispatch.launch.py
```

기본값은 거리 비교 전용입니다. RViz의 **Publish Point**로 좌표를 클릭하면
로봇별 Nav2 경로거리와 선택 결과를 발행합니다.

```bash
ros2 topic echo /aed/path_distance/robot1
ros2 topic echo /aed/path_distance/robot2
ros2 topic echo /aed/selected_robot
```

## 중앙 노트북 통합 실행

Nav2 두 대와 골목 카메라를 제외한 중앙 노트북 프로세스는 다음 명령 하나로
실행합니다. 경보, HMI 백엔드·프론트엔드, 개방구역 카메라, 중앙 미션,
로봇별 preview Vision과 helper 노드가 포함됩니다.

```bash
ros2 launch aed_bringup server_runtime.launch.py
```

기본값은 실제 출동을 막아 둡니다. 실제 주행 시에만 명시적으로 켭니다.

```bash
ros2 launch aed_bringup server_runtime.launch.py dispatch_enabled:=true
```

`tools/aliases.sh`를 불러온 터미널에서는 기존과 동일하게 다음 단축 명령을
사용할 수 있습니다. 이 명령도 이제 개별 중앙 launch가 아니라 위 통합
런타임 전체를 실행합니다.

```bash
central true
```

같은 PC에서 이 launch를 두 번 실행하면 두 번째 실행은 노드를 만들기 전에
종료됩니다. 기존 개별 launch와 함께 실행하면 잠금으로 잡을 수 없으므로,
통합 launch를 사용할 때는 HMI·경보·카메라·중앙 launch를 따로 실행하지
않습니다.

필요한 구성만 끌 수도 있습니다.

```bash
ros2 launch aed_bringup server_runtime.launch.py \
  start_frontend:=false \
  start_open_camera:=false \
  start_robot_vision:=false
```
