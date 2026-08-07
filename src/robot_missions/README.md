# robot_missions

TurtleBot4가 Mission Manager의 명령을 받아 Nav2 임무를 수행하는 ROS 2 Humble
Python 패키지입니다.

이 패키지에는 다음 두 실행 노드가 있습니다.

| 노드 | 역할 | 실행 방식 |
| --- | --- | --- |
| `mission_executor` | AED 전달 목표를 `NavigateToPose`로 실행 | `ros2 run` |
| `search_and_detect_node` | Polygon 지그재그 수색, YOLO 탐지 시 목표 접근 | Python 파일 직접 실행 |

> `search_and_detect_node`는 현재 단일 파일 형태로 추가되어 `setup.py`의
> console script에는 아직 등록되지 않았습니다. 따라서 아래 설명처럼 워크스페이스
> 루트에서 Python 파일을 직접 실행해야 합니다.

## Search and Detect 동작 흐름

1. Mission Manager가 수색 Polygon 꼭짓점을 Action Goal로 보냅니다.
2. 노드는 Polygon을 map 좌표로 변환하고 Boustrophedon(지그재그) 경로를
   생성합니다.
3. Nav2 `NavigateThroughPoses`로 생성된 경로를 주행합니다.
4. 수색 중 YOLO 쓰러짐 감지가 확정되면 coverage Goal을 즉시 취소합니다.
5. 감지 위치에서 `approach_distance_m`만큼 떨어진 접근점을 계산합니다.
6. Nav2 `NavigateToPose`로 대상 앞까지 이동하고 Mission Manager Action을
   성공 처리합니다.
7. 전체 수색 후 미탐지, Nav2 오류, TF timeout은 Action abort로 반환합니다.

상태는 다음 순서로 전환됩니다.

```text
IDLE
  └─> SEARCHING_COVERAGE
        ├─> PERSON_DETECTED ─> APPROACHING_TARGET ─> ARRIVED
        └─> SEARCH_FAILED
```

## 1. 실행 환경 준비

권장 환경은 Ubuntu 22.04, ROS 2 Humble, Python 3, TurtleBot4/Nav2입니다.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-tf2-geometry-msgs \
  python3-colcon-common-extensions \
  python3-rosdep
```

ROS 환경을 불러오고 누락된 패키지 의존성을 설치합니다.

```bash
cd ~/aed_bot_ws
source /opt/ros/humble/setup.bash

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`aed_interfaces`가 먼저 생성되도록 관련 패키지를 빌드합니다.

```bash
colcon build --symlink-install \
  --packages-select aed_interfaces robot_missions
source install/setup.bash
```

새 터미널을 열 때마다 다음 환경 설정이 필요합니다.

```bash
cd ~/aed_bot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. 실행 전 확인 사항

Search and Detect 노드를 실행하기 전에 해당 로봇 namespace에서 다음 구성요소가
동작해야 합니다.

- Nav2 `navigate_through_poses` Action Server
- Nav2 `navigate_to_pose` Action Server
- `map`에서 `base_link`로 이어지는 TF
- YOLO 검출 결과 publisher
- AMCL 또는 SLAM 기반 localization

예를 들어 `/amr_1`을 실행한다면 다음 명령으로 Nav2 Action을 확인합니다.

```bash
ros2 action list | grep '/amr_1/navigate'
```

정상적인 예시는 다음과 같습니다.

```text
/amr_1/navigate_through_poses
/amr_1/navigate_to_pose
```

TF도 확인합니다.

```bash
ros2 run tf2_ros tf2_echo map base_link
```

로봇별 TF가 `amr_1/base_link`처럼 prefix된 구성이라면 실행 파라미터
`base_frame`도 그 이름으로 변경해야 합니다.

## 3. Search and Detect 노드 실행

워크스페이스 루트에서 실행합니다. Action과 토픽 기본값은 상대 이름이므로
namespace가 `/amr_1`이면 자동으로 `/amr_1/...` 아래에 생성됩니다.

### AMR 1

```bash
cd ~/aed_bot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/robot_missions/robot_missions/search_and_detect_node.py \
  --ros-args \
  -r __ns:=/amr_1
```

### AMR 2

```bash
cd ~/aed_bot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/robot_missions/robot_missions/search_and_detect_node.py \
  --ros-args \
  -r __ns:=/amr_2
```

노드가 정상적으로 시작되면 다음과 비슷한 로그가 출력됩니다.

```text
SearchAndDetect ready: namespace=/amr_1 ...
```

### 현재 프로젝트 YOLO 토픽과 연결

기본 검출 토픽은 namespace 상대 이름인 `vision/emergency_event`입니다. 현재
`aed_vision` 고정 카메라 노드는 다음과 같은 절대 토픽을 사용합니다.

- `/camera_open/vision/emergency_event`
- `/camera_alley/vision/emergency_event`

`camera_open` 결과를 AMR 1에 연결하려면 다음처럼 파라미터를 덮어씁니다.

```bash
python3 src/robot_missions/robot_missions/search_and_detect_node.py \
  --ros-args \
  -r __ns:=/amr_1 \
  -p detection_event_topic:=/camera_open/vision/emergency_event
```

YOLO가 프레임별 `DetectionSummary`를 발행한다면 선택 토픽을 추가할 수 있습니다.

```bash
-p detection_summary_topic:=vision/detection_summary
```

두 검출 입력이 동시에 설정되면 먼저 임계값을 통과한 대상 하나만 latch합니다.

## 4. Mission Manager에서 Polygon Goal 보내기

프로젝트에는 아직 Polygon 전용 Action 정의가 없습니다. 따라서 Mission
Manager용 Action Server도 `nav2_msgs/action/NavigateThroughPoses`를 사용하며,
`goal.poses` 배열을 **주행 waypoint가 아닌 Polygon 꼭짓점**으로 해석합니다.

- Mission Manager Action: `/amr_1/search_and_detect`
- Nav2 coverage Action: `/amr_1/navigate_through_poses`
- Nav2 approach Action: `/amr_1/navigate_to_pose`

Mission Manager는 Polygon 꼭짓점을 시계 방향 또는 반시계 방향으로 보내야
합니다. 마지막 꼭짓점을 첫 점과 중복해서 닫는 것은 선택 사항입니다.

다음은 map 좌표 `(0, 0)`, `(3, 0)`, `(3, 2)`, `(0, 2)` 영역을 수색하는
수동 시험 명령입니다.

```bash
ros2 action send_goal \
  /amr_1/search_and_detect \
  nav2_msgs/action/NavigateThroughPoses \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 0.0, y: 2.0}, orientation: {w: 1.0}}}
  ], behavior_tree: ''}" \
  --feedback
```

Action 결과는 다음 의미를 가집니다.

| Action 상태 | 의미 |
| --- | --- |
| `SUCCEEDED` | 쓰러진 사람을 탐지하고 접근점까지 도착 |
| `ABORTED` | 미탐지 수색 완료, Nav2 실패, TF 실패 또는 timeout |
| `CANCELED` | Mission Manager가 임무 취소 |

## 5. YOLO 검출 수동 주입

수색 Goal이 실행 중일 때 다른 터미널에서 다음 이벤트를 발행하면 탐지→취소→접근
전환을 확인할 수 있습니다.

```bash
ros2 topic pub --once \
  /amr_1/vision/emergency_event \
  aed_interfaces/msg/EmergencyEvent \
  "{event_id: test-fallen-001,
    location: {
      header: {frame_id: map},
      point: {x: 1.8, y: 0.8, z: 0.0}
    },
    confidence: 0.95,
    consecutive_detections: 1,
    status: 1,
    source_id: manual-test,
    camera_id: mock-yolo,
    zone_id: test-zone,
    crowd_level: 0}"
```

`status: 1`은 현재 `EmergencyEvent.CONFIRMED`입니다. 기본 설정에서는 confidence
`0.60` 이상인 `DETECTED` 또는 `CONFIRMED` 이벤트를 수용합니다.

성공적인 전환 로그는 다음 순서로 나타납니다.

```text
IDLE -> SEARCHING_COVERAGE
SEARCHING_COVERAGE -> PERSON_DETECTED
PERSON_DETECTED -> APPROACHING_TARGET
APPROACHING_TARGET -> ARRIVED
```

## 6. 내장 Mock 모드

`--mock` 옵션은 Mock Polygon Action Goal과 Mock YOLO 이벤트를 같은 프로세스에서
자동으로 주입합니다.

```bash
python3 src/robot_missions/robot_missions/search_and_detect_node.py \
  --mock \
  --ros-args \
  -r __ns:=/amr_1 \
  -p mock_detection_delay_s:=3.0
```

기본 Mock 데이터는 다음과 같습니다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `mock_polygon_xy` | `[-2,-1.5, 2,-1.5, 2,1.5, -2,1.5]` | Polygon x,y 배열 |
| `mock_detection_xy` | `[0.8, 0.4]` | 감지 대상 map 좌표 |
| `mock_detection_confidence` | `0.95` | 가상 YOLO confidence |
| `mock_detection_delay_s` | `3.0` | Goal 수락 후 탐지 발행 지연 |

Mock 모드는 Mission Manager와 YOLO 입력만 대체합니다. 실제 이동 결과를 받으려면
Nav2 Action Server, map localization, TF는 실제 로봇 또는 시뮬레이터에서 실행되어
있어야 합니다.

## 7. 주요 파라미터

모든 이름과 수색 설정은 ROS 2 파라미터로 변경할 수 있습니다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `search_action_name` | `search_and_detect` | Mission Manager용 Action 이름 |
| `detection_event_topic` | `vision/emergency_event` | YOLO `EmergencyEvent` 토픽 |
| `detection_summary_topic` | 빈 문자열 | 선택적인 `DetectionSummary` 토픽 |
| `navigate_through_poses_action` | `navigate_through_poses` | Nav2 coverage Action |
| `navigate_to_pose_action` | `navigate_to_pose` | Nav2 target approach Action |
| `map_frame` | `map` | Polygon과 접근 목표의 기준 frame |
| `base_frame` | `base_link` | 현재 로봇 위치를 조회할 frame |
| `camera_fov_deg` | `69.0` | 로봇 카메라 수평 FOV |
| `detection_range_m` | `2.0` | FOV 기반 관측 폭 계산 거리 |
| `tool_width_m` | `0.6` | 최대 coverage lane 폭 |
| `path_overlap_ratio` | `0.20` | 인접 카메라 관측 영역 겹침률 |
| `sweep_angle_deg` | `0.0` | map x축 기준 지그재그 진행 각도 |
| `boundary_margin_m` | `0.10` | Polygon 경계로부터 endpoint 여유 |
| `minimum_lane_length_m` | `0.20` | 너무 짧은 lane 제거 기준 |
| `max_waypoints` | `0` | 최대 waypoint 수, 0은 무제한 |
| `detection_confidence_threshold` | `0.60` | 최소 YOLO confidence |
| `detection_required_hits` | `1` | target latch에 필요한 연속 검출 수 |
| `detection_reset_timeout_s` | `1.0` | 연속 검출 수 초기화 간격 |
| `detection_max_age_s` | `2.0` | 오래된 검출 메시지 폐기 기준 |
| `approach_distance_m` | `0.70` | 대상과 최종 정지점 사이 거리 |
| `nav2_server_timeout_s` | `3.0` | Nav2 Action Server 대기시간 |
| `tf_timeout_s` | `0.5` | Polygon/대상 좌표 TF 제한시간 |
| `search_timeout_s` | `180.0` | coverage 제한시간, 0은 무제한 |

예를 들어 lane 폭을 좁히고 탐지 신뢰도 기준을 높이려면 다음처럼 실행합니다.

```bash
python3 src/robot_missions/robot_missions/search_and_detect_node.py \
  --ros-args \
  -r __ns:=/amr_1 \
  -p tool_width_m:=0.4 \
  -p detection_confidence_threshold:=0.75 \
  -p approach_distance_m:=0.8
```

## 8. 문제 해결

### `ModuleNotFoundError: nav2_msgs` 또는 ROS 메시지 import 실패

ROS 환경과 워크스페이스를 source했는지 확인합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/aed_bot_ws/install/setup.bash
```

그 후 `aed_interfaces`와 `robot_missions`를 다시 빌드합니다.

### `Nav2 NavigateThroughPoses action server unavailable`

해당 namespace에서 Nav2가 활성화되지 않았거나 Action 이름이 다릅니다.

```bash
ros2 action list | grep navigate
```

출력된 이름에 맞춰 `navigate_through_poses_action` 또는
`navigate_to_pose_action`을 변경합니다. 절대 Action 이름도 사용할 수 있습니다.

### `polygon TF timeout` 또는 `target TF/approach calculation failed`

Polygon/YOLO 메시지의 `frame_id`, `map_frame`, `base_frame`이 실제 TF tree와
일치하는지 확인합니다.

```bash
ros2 run tf2_ros tf2_echo map base_link
```

센서 timestamp가 너무 과거라면 해당 시각의 TF가 buffer에서 사라졌을 수 있습니다.

### 수색을 끝냈지만 Action이 `ABORTED`

오류가 아니라 Polygon 전체를 주행할 때까지 쓰러진 사람을 찾지 못한
`Search Failed` 결과일 수 있습니다. 노드 로그의 `Search failed:` 상세 이유를
확인합니다.

### 검출 토픽이 있는데 반응하지 않음

다음을 확인합니다.

- 노드가 `SEARCHING_COVERAGE` 상태인지
- 이벤트 status가 `DETECTED(0)` 또는 `CONFIRMED(1)`인지
- confidence가 `detection_confidence_threshold` 이상인지
- `location.header.frame_id`가 비어 있지 않은지
- 검출 timestamp가 `detection_max_age_s`보다 오래되지 않았는지
- 실제 토픽 이름과 `detection_event_topic` 설정이 같은지

```bash
ros2 topic echo /amr_1/vision/emergency_event
ros2 topic info /amr_1/vision/emergency_event --verbose
```

### 새 Polygon Goal이 거절됨

동시에 하나의 수색 임무만 실행할 수 있습니다. 기존 Action을 취소하거나 종료한
후 다시 전송하십시오. Polygon에는 서로 다른 꼭짓점이 최소 3개 필요합니다.

## 기존 AED Mission Executor

기존 `mission_executor`는 로봇별 `MissionAssignment`를 받아 AED 전달 목표를
Nav2 `NavigateToPose`로 실행하고 typed `MissionStatus`를 발행합니다.

- 새로운 assignment version을 수신하면 기존 Goal 취소
- 출동·주행·도착·취소·Nav2 실패 상태 보고
- 실패 후 자체적으로 이전 Goal을 재개하지 않음

```bash
ros2 run robot_missions mission_executor --ros-args \
  -r __ns:=/robot1 \
  -p robot_id:=robot1
```

## 빠른 정적 검사

ROS 런타임이 없는 개발 PC에서도 Python 문법은 확인할 수 있습니다.

```bash
python3 -m py_compile \
  src/robot_missions/robot_missions/search_and_detect_node.py
```
