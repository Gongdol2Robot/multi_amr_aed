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

`mapnav 1`, `mapnav 2`는 이제 각 로봇의 map navigation과
`sensor_recovery` LiDAR watchdog/fallback을 같은 launch에서 실행합니다.
기존처럼 localization과 Nav2를 나눠 실행할 때도 `nav 1`, `nav 2`가 Nav2와
fallback을 함께 시작합니다. 이미 Nav2가 실행 중인 진단 상황에서는
`fallback 1`처럼 recovery만 별도로 실행할 수 있지만 중복 실행하면 안 됩니다.

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

### 비전 backend 선택

통합 런타임의 비전 backend는 최상위 `vision_backend` 인자 하나로 정합니다.
이 값은 개방구역 고정 카메라와 robot1·robot2 비전 노드에 함께 전달됩니다.
기본값은 목각인형을 검출하는 `mannequin`입니다.

실제 사람 모드에서는 Pose 모델이 쓰러진 사람의 자세를 판정하고, 별도 COCO
person 모델이 같은 프레임의 다른 사람을 찾아 `helping_person` 후보로 사용합니다.

```bash
ros2 launch aed_bringup server_runtime.launch.py \
  vision_backend:=person_pose
```

목각인형 모드는 파인튜닝 구조 모델과 목각인형 자세 분류기를 사용합니다.

```bash
ros2 launch aed_bringup server_runtime.launch.py \
  vision_backend:=mannequin
```

고정 카메라 없이 중앙 배차와 두 로봇 비전만 직접 실행할 때도 같은 이름의
인자를 사용합니다.

```bash
ros2 launch multi_robot_emergency central_dispatch.launch.py \
  vision_backend:=person_pose

ros2 launch multi_robot_emergency central_dispatch.launch.py \
  vision_backend:=mannequin
```

고정 USB 카메라만 개별 시험할 때는 하위 launch의 인자명이 `backend`입니다.

```bash
ros2 launch aed_vision camera_vision.launch.py \
  camera:=1 backend:=person_pose

ros2 launch aed_vision camera_vision.launch.py \
  camera:=1 backend:=mannequin
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

`central`은 HMI를 포함한 중앙 PC 통합 런타임 명령입니다. 첫 번째 인자
`true/false`는 HMI 실행 여부가 아니라 실제 로봇 출동 허용 여부입니다.
목각인형이 기본 backend이므로 기존 명령을 그대로 사용합니다. 실제 사람
Pose 모드만 `centralp`로 구분합니다.

```bash
central false           # HMI 포함 전체 런타임, 목각인형, 출동 비활성
central true            # HMI 포함 전체 런타임, 목각인형, 실제 출동
centralp false          # HMI 포함 전체 런타임, 실제 사람, 출동 비활성
centralp true           # HMI 포함 전체 런타임, 실제 사람, 실제 출동

visionperson 1          # 고정 카메라 1만 실제 사람 모드로 실행
visionmannequin 1       # 고정 카메라 1만 목각인형 모드로 실행
```

기존 호환 단축어 `centralperson`, `centralmannequin`, `cperson`,
`cmannequin`도 유지합니다. 고정 카메라는 `vperson`, `vmannequin`으로 더
짧게 실행할 수 있습니다. `central` 명령은 다섯 번째 인자로 backend를 받을
수 있으며, 생략 시 `mannequin`을 사용합니다.

```bash
central false 30.0 0.85 true person_pose
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
