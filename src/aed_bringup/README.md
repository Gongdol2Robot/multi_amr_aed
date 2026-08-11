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

각 로봇 PC에서 Nav2를 따로 실행한 뒤 중앙 PC에서 실제 Nav2 경로와 ETA를
비교해 미션을 배정합니다. 중앙 런치는 Nav2나 RViz를 실행하지 않습니다.

```bash
ros2 launch multi_robot_emergency central_dispatch.launch.py
```

기본값은 계산 전용입니다. RViz의 **Publish Point**로 좌표를 클릭하거나 비전이
`CONFIRMED` 이벤트를 발행하면 로봇별 Nav2 경로거리와 선택 결과를 발행합니다.

```bash
ros2 topic echo /emergency/path_distance/robot1
ros2 topic echo /emergency/path_distance/robot2
ros2 topic echo /emergency/selected_robot
```

## 중앙 노트북 통합 실행

중앙 노트북의 경보, HMI 백엔드·프론트엔드, 중앙 미션과 helper 노드는 다음
명령 하나로 실행합니다. 카메라와 `vision_detector`는 포함하지 않으며, 다른
노트북이 발행하는 ROS 2 검출 결과와 압축 영상을 구독합니다.

```bash
ros2 launch aed_bringup server_runtime.launch.py
```

### 비전 검출기 분리 실행

비전 backend는 중앙 PC가 아니라 각 `vision_detector`를 실행하는 노트북에서
선택합니다. 고정 카메라 노트북에서는 다음 중 하나를 실행합니다.

```bash
ros2 launch aed_vision camera_vision.launch.py \
  camera:=1

ros2 launch aed_vision camera_vision.launch.py \
  camera:=2
```

robot1·robot2 검출기는 서로의 추론을 기다리지 않도록 별도 프로세스로
실행합니다.

```bash
ros2 launch aed_vision robot_vision.launch.py \
  robot_id:=robot1

ros2 launch aed_vision robot_vision.launch.py \
  robot_id:=robot2
```

위 기본 명령은 USB 웹캠과 OAK-D 모두 `backend:=mannequin`을 적용해
`rescue2_yolo11n.pt`로 낙상 대상과 helping RC카(`helping_person` class)를
함께 검출합니다. 필요할 때만 명시적으로 `backend:=person_pose`를 붙여 실제
사람 자세 인식 경로로 바꿀 수 있습니다. RC카는 같은 프레임에 낙상 대상이
가까이 있을 때만 확정되며, 로봇의
`/robotN/vision/helper_confirmed=true`가 helper 인계·복귀 절차를 시작합니다.

로봇용 launch는 시작 직후에는 배정 토픽만 구독합니다. 해당
`/robotN/mission_assignment`를 받은 뒤에만 OAK-D preview 구독을 생성하므로,
배정 전에는 로봇 프레임이 비전 노트북으로 전송되거나 추론되지 않습니다.
배송 도착 뒤 조력자 탐색까지는 추론을 유지하고, 탐색 종료·실패 또는 복귀
도착 상태를 받으면 이미지 구독을 제거합니다.

모든 노트북은 같은 `ROS_DOMAIN_ID`와 discovery server를 사용해야 합니다.

통합 런타임은 현재 실제 출동이 기본으로 활성화되어 있습니다. 화면과 비전만
시험할 때는 명시적으로 끕니다.

```bash
ros2 launch aed_bringup server_runtime.launch.py dispatch_enabled:=false
```

`tools/aliases.sh`를 불러온 터미널에서는 기존과 동일하게 다음 단축 명령을
사용할 수 있습니다. 이 명령도 이제 개별 중앙 launch가 아니라 위 통합
런타임 전체를 실행합니다.

```bash
central true
```

`central`은 HMI를 포함한 중앙 PC 통합 런타임 명령입니다. 첫 번째 인자
`true/false`는 HMI 실행 여부가 아니라 실제 로봇 출동 허용 여부입니다.
비전 backend는 원격 비전 노트북에서 선택하므로 `central`의 인자가 아닙니다.
`centralp`, `centralm`은 기존 사용법 호환을 위해 남아 있지만 중앙에서는
동일하게 동작합니다.

인자 없이 `central`, `centralp`, `centralm`을 실행하면 실제 출동이
활성화됩니다. 비주행 시험에서는 반드시 첫 번째 인자로 `false`를 줍니다.

```bash
central false           # HMI 포함 중앙 런타임, 출동 비활성
central true            # HMI 포함 중앙 런타임, 실제 출동
centralp false          # central false와 동일한 호환 명령
centralm true           # central true와 동일한 호환 명령

visionperson 1          # 고정 카메라 1만 실제 사람 모드로 실행
visionmannequin 1       # 고정 카메라 1만 목각인형 모드로 실행
```

`central`은 중앙 PC에서 USB 카메라와 robot1·robot2 `vision_detector`를 모두
실행하지 않습니다. 같은 Wi-Fi/ROS 2 도메인의 비전 노트북이 발행하는 압축
검출 영상을 HMI로 받습니다.

```text
/camera_open/vision/debug/compressed
/camera_alley/vision/debug/compressed
/robot1/vision/debug/compressed
/robot2/vision/debug/compressed
```

카메라 1 노트북에서는 `visionmannequin 1` 또는 `visionperson 1`을 실행해야
하며, 중앙 PC와 같은 `ROS_DOMAIN_ID`와 discovery server를 사용해야 합니다.

기존 호환 단축어 `centralperson`, `centralmannequin`, `cperson`,
`cmannequin`도 유지합니다. 고정 카메라는 비전 노트북에서 `vperson`,
`vmannequin`으로 더 짧게 실행할 수 있습니다.

같은 PC에서 이 launch를 두 번 실행하면 두 번째 실행은 노드를 만들기 전에
종료됩니다. 기존 개별 launch와 함께 실행하면 잠금으로 잡을 수 없으므로,
통합 launch를 사용할 때는 HMI·경보·중앙 launch를 따로 실행하지 않습니다.

프론트엔드처럼 중앙 구성 일부만 끌 수도 있습니다. 비전 검출기는 중앙 launch의
구성 항목이 아니므로 `camera_vision.launch.py`나 `robot_vision.launch.py`로
다른 노트북에서 별도 실행합니다.

```bash
ros2 launch aed_bringup server_runtime.launch.py \
  start_frontend:=false
```
