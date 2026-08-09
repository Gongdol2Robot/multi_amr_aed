# aed_vision

고정 USB 웹캠 영상에서 쓰러진 구조 대상과 구조 보조자(`helper`)를 검출하고,
골목 카메라에서는 실제 사람 수를 이용해 통로 혼잡도까지 판단하는 ROS 2
패키지입니다.

담당: 김지훈(호모그래피·위치 검증), 이현민(구조 대상·사람 검출 및 통합)

## 카메라 구성

같은 코드를 두 노트북에 설치하고 YAML 설정만 다르게 실행합니다. 두 노트북은
같은 ROS 2 네트워크와 `ROS_DOMAIN_ID`를 사용해야 합니다.

| 카메라 | 모드 | 기본 검출 | 선택 검출 | 혼잡도 |
|---|---|---|---|---|
| `camera_open` | `open` | 실제 사람 Pose | 목각인형 파인튜닝 | `NOT_APPLICABLE` |
| `camera_alley` | `alley` | 실제 사람 Pose | 목각인형 파인튜닝 | `CLEAR`/`CROWDED` |

- `open`: 탁 트인 장소에서 `fallen_person`과 `helper`만 검출합니다.
- `alley`: 좁은 통로에서 구조 검출과 ROI 내부 실제 사람 수를 함께 계산합니다.
- 학습 가중치 내부 클래스명은 `helper_rc_car`지만 디버그 bbox와 상태 JSON에는
  역할 중심 이름인 `helper`를 사용합니다.

## 노드

### `vision_detector`

- 각 노트북의 USB 웹캠을 직접 읽어 같은 프로세스에서 즉시 추론
- 읽은 원본 영상은 모니터링용 JPEG 압축 토픽으로도 발행
- 기본 `person_pose` backend는 YOLO11n-Pose의 실제 사람 관절과 bbox로 자세 판정
- 선택 `mannequin_detect` backend는 기존 파인튜닝 YOLO11n 검출을 그대로 사용
- 최근 10프레임 중 6프레임 이상 검출될 때 응급상황 확정
- 확정/해제 전환 시 `aed_interfaces/EmergencyEvent` 발행
- 검출 bbox 하단 중심점을 카메라별 호모그래피로 map 좌표 변환
- `alley` 모드에서는 COCO YOLO11n으로 ROI 내부 `person` 수 계산
- COCO가 쓰러진 대상을 person으로 중복 검출하면 bbox IoU를 이용해 인파에서 제외
- 상태 JSON, 혼잡도, 사람 수, heartbeat, JPEG 디버그 영상 발행
- 기본 설정에서는 실행한 노트북에 OpenCV 실시간 검출 창 표시

한 프레임을 처리하는 동안 새 프레임이 도착하면 큐 깊이 1의 best-effort QoS를
사용해 오래된 프레임을 쌓지 않습니다.

실행 launch는 카메라별 `vision_detector` 노드 하나만 시작합니다.

### 검출 confidence를 0.25로 둔 이유

구조 대상 검출의 기본 confidence 임계값인 `rescue_conf`는 `0.25`입니다.
응급상황에서는 오검출보다 쓰러진 사람을 놓치는 미검출의 비용이 더 크므로,
1차 YOLO 검출에서는 재현율(recall)을 우선해 후보를 넓게 받도록 설정했습니다.

낮은 임계값에서 발생할 수 있는 순간적인 오검출을 곧바로 응급상황으로 처리하지는
않습니다. 각 프레임에서 confidence가 `0.25` 이상인 `fallen_person`을 후보로 받고,
최근 10프레임 중 6프레임 이상에서 후보가 검출될 때만 응급상황을 확정합니다.
이 6프레임은 연속일 필요가 없습니다.

```text
YOLO confidence >= 0.25
        ↓ 후보 검출 — recall 우선
최근 10프레임 중 6프레임 이상 검출
        ↓ 시간적 검증 — 순간 오검출 억제
EmergencyEvent.CONFIRMED
```

따라서 `0.25` 하나만으로 응급상황을 확정하는 구조가 아니라, 낮은 임계값으로
후보를 확보한 뒤 다중 프레임 검증으로 신뢰도를 보완하는 구조입니다. 다만
`0.25`는 현재 기본 설정값이며 현장 데이터로 최적값이 검증된 수치는 아닙니다.
최종 배포 전에는 실제 카메라 위치·조명·거리에서 precision, recall, 오검출률과
미검출률을 측정해 `rescue_conf`, `confirmation_window`, `confirmation_hits`를 함께
조정해야 합니다.

## 설치

```bash
cd ~/rokey_ws/multi_amr_aed
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r src/aed_vision/requirements.txt
colcon build --packages-select aed_interfaces aed_vision
source install/setup.bash
```

세 모델은 패키지의 `models/`에 포함되고 빌드할 때 ROS share 폴더에 함께
설치됩니다.

- `models/rescue2_yolo11n.pt`: 파인튜닝 구조 검출 모델
- `models/coco_yolo11n.pt`: COCO person 검출 모델
- `models/yolo11n-pose.pt`: 실제 사람 17관절 Pose 모델

YAML은 절대 경로 대신 다음 ROS 패키지 URI를 사용하므로 노트북마다 경로를
수정할 필요가 없습니다.

```text
package://aed_vision/models/rescue2_yolo11n.pt
package://aed_vision/models/coco_yolo11n.pt
package://aed_vision/models/yolo11n-pose.pt
```

## 실행

탁 트인 공간 노트북:

```bash
ros2 launch aed_vision camera_vision.launch.py \
  camera:=1
```

`camera:=1`은 `camera_open` namespace와 `open_camera.yaml`을 자동 선택하며,
기본적으로 목각인형 파인튜닝 모델로 쓰러짐을 판정합니다. 실행한
노트북에는 `AED Vision - camera_open (open)` 결과 창이 표시됩니다.

실제 사람 Pose 판정을 명시적으로 선택하려면 다음처럼 실행합니다.

```bash
ros2 launch aed_vision camera_vision.launch.py \
  camera:=1 target:=person
```

backend별 confidence도 launch에서 조절할 수 있습니다.

```bash
# 실제 사람 Pose: 기본 0.5
ros2 launch aed_vision camera_vision.launch.py \
  camera:=1 person_conf:=0.55

# 목각인형 파인튜닝: 기본 0.25
ros2 launch aed_vision camera_vision.launch.py \
  camera:=1 target:=mannequin rescue_conf:=0.30
```

두 backend는 다음 두 값만 허용합니다.

- `person_pose`: 실제 사람의 bbox·17관절을 검출하고 종횡비와 몸통 각도로
  `STANDING`, `SITTING`, `FALLEN` 판정
- `mannequin_detect`(기본): `rescue2_yolo11n.pt`의 목각인형 기반
  `fallen_person`, `helper_rc_car` 검출

좁은 골목 노트북:

```bash
ros2 launch aed_vision camera_vision.launch.py \
  camera:=2
```

`camera:=2`는 `camera_alley` namespace와 `alley_camera.yaml`을 자동 선택하며,
쓰러진 사람과 helper 검출에 COCO person 기반 인파 감지를 추가합니다. 실행한
노트북에는 구조 bbox, person bbox와 혼잡 ROI가 합쳐진 결과 창이 표시됩니다.

실행 전 각 YAML에서 다음 값을 현장에 맞게 수정합니다.

- `camera_device`: USB 웹캠 장치 경로. 기본값 `auto`는
  `/dev/v4l/by-id`에서 노트북 내장 카메라를 제외한 외장 USB 웹캠을 선택하며, 가능하면
  재부팅 후에도 유지되는 `/dev/v4l/by-id/...` 경로 사용 권장
- `inference_device`: YOLO 추론 장치. `"cuda:0"`은 첫 GPU를 우선 사용하되
  CUDA 또는 해당 GPU가 없으면 경고 후 CPU로 전환한다. `"cpu"`는 CPU 고정,
  빈 문자열은 Ultralytics의 자동 장치 선택을 사용한다.
- `target`: `person`(기본) 또는 `mannequin`
- `pose_weights`: 실제 사람용 Pose 가중치
- `person_conf`, `pose_keypoint_conf`, `pose_min_keypoints`,
  `pose_min_box_area`: Pose 사람·관절 품질 필터
- `rescue_conf`: 구조 대상 YOLO의 1차 후보 confidence 임계값. 기본값은 `0.25`
- `location_x`, `location_y`: 해당 고정 카메라 구조 지점의 map 좌표
- `homography_camera_id`: 카메라별 측량 설정 ID (`cam1` 또는 `cam2`)
- `homography_margin_m`: 측량 영역 경계에서 허용할 좌표 여유
- `crowd_roi`: 골목 영상에서 AMR이 통과해야 하는 영역
- `show_window`: 해당 노트북에 OpenCV 결과 창을 표시할지 여부

## 토픽

`<camera_id>`는 `camera_open` 또는 `camera_alley`입니다.

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/<camera_id>/image_raw/compressed` | `sensor_msgs/CompressedImage` | 로컬 USB 웹캠 JPEG |
| `/<camera_id>/vision/emergency_event` | `aed_interfaces/EmergencyEvent` | 확정 또는 해제된 구조 이벤트 |
| `/<camera_id>/vision/status` | `std_msgs/String` | 전체 검출 상태 JSON |
| `/<camera_id>/vision/crowd_level` | `aed_interfaces/CrowdLevel` | 0~3 혼잡 등급과 사람 수·통행 가능 여부 |
| `/<camera_id>/vision/person_count` | `std_msgs/UInt32` | 골목 ROI 내 유효 person 수 |
| `/<camera_id>/vision/detection_summary` | `aed_interfaces/DetectionSummary` | 프레임별 구조화된 검출 요약 |
| `/<camera_id>/vision/fallen_location` | `geometry_msgs/PointStamped` | 호모그래피로 계산한 구조 대상 map 좌표 |
| `/<camera_id>/vision/heartbeat` | `aed_interfaces/Heartbeat` | 초당 노드 생존 신호 |
| `/<camera_id>/vision/debug/compressed` | `sensor_msgs/CompressedImage` | bbox와 ROI가 표시된 JPEG |

로봇 OAK-D 모드는 로봇에서 이미 발행하는 JPEG 압축 영상을 구독합니다.

```text
/robotN/oakd/rgb/image_raw/compressed  (sensor_msgs/CompressedImage)
```

노트북에서 JPEG를 디코딩한 뒤 `robot_camera.yaml`에 설정된 추론 파이프라인에
입력합니다. 로봇 비전의 backend, confidence와 조력자 판정 기준은 launch 인자가
덮어쓰지 않으며 이 YAML을 단일 기준으로 사용합니다.
라즈베리파이에는 모델이나 추가 추론 프로세스를 설치하지 않습니다.

로봇의 조력자 후보는 환자와 같은 프레임에 있어야 하며, 조력자 bbox 하단
중심과 쓰러진 환자 bbox 중심 사이 거리가 화면 대각선의 30% 이내여야 합니다.
이 값은 `helper_max_distance_ratio`로 조정할 수 있습니다. 거리 조건을 통과한
후에도 최근 6개 처리 프레임 중 3개 이상에서 보여야 최종 확정됩니다.

`status` JSON 예시:

```json
{
  "camera_id": "camera_alley",
  "zone_id": "alley_zone",
  "mode": "alley",
  "detection_backend": "person_pose",
  "fallen_detected": true,
  "fallen_confirmed": true,
  "fallen_count": 1,
  "fallen_max_confidence": 0.91,
  "posture": "FALLEN",
  "pose_aspect_ratio": 1.91,
  "pose_torso_angle_deg": 19.4,
  "pose_visible_keypoints": 14,
  "helper_count": 0,
  "person_count": 3,
  "crowd_level": 3,
  "crowd_time_multiplier": null,
  "crowd_traversable": false,
  "confirmation_hits": 7,
  "inference_ms": 42.5
}
```

## 혼잡도 ROI 조정

`crowd_roi`는 `[left, top, right, bottom]` 순서이며 영상 너비·높이에 대한
0.0~1.0 비율입니다. 현재 골목 카메라는 `[0.375, 0.0, 1.0, 0.5]`로 영상
상단의 오른쪽 62.5% 영역을 사용합니다. 이 영역은 1번 터틀봇의 골목 진입
경로입니다. 실제 이동 통로에 맞게 디버그 영상의 청록색 사각형을 보면서
조정해야 합니다.

혼잡 등급은 쓰러진 대상 외의 실제 사람 수를 사용합니다. 파인튜닝
모델의 `fallen_person` bbox와 COCO YOLO11n의 `person` bbox가 겹치면 동일한
쓰러진 대상으로 판단해 사람 수에서 제외합니다. 제외 후 우상단 ROI에 남은
person이 0명이면 시간 패널티가 없고, 1명이면 10%, 2명이면 20%를 더합니다.
3명 이상은 모두 3등급이며 이동 불가로 판단합니다. `status` JSON의
`crowd_time_multiplier`는 각각 `1.0`, `1.1`, `1.2`, `null`이고,
`crowd_traversable`은 3등급부터 `false`입니다.

혼잡도는 골목 카메라에서만 의미가 있으므로 `camera_open`은 항상
`NOT_APPLICABLE`을 발행합니다. 중앙 Mission Manager는 이 값을 사람 수 0과
구별해야 합니다.

## 검출 위치 좌표

각 카메라는 `homography_cam1.yaml` 또는 `homography_cam2.yaml`의 현장 측량
행렬을 사용합니다. 가장 confidence가 높은 `fallen_person` bbox의 하단 중심점을
`map` 좌표로 변환하여 `EmergencyEvent.location`에 넣습니다.

입력 영상 해상도가 측량 당시의 640×480과 다르면 픽셀 좌표를 측량 해상도로
자동 환산합니다. 측량 영역에서 `homography_margin_m`보다 멀리 벗어난 검출은
측량 영역 밖의 검출도 폐기하지 않고 외삽 좌표로 발행합니다. 프레임별 좌표와
변환 방식은 `vision/status` JSON의 `location_x`, `location_y`,
`location_source`에서 확인할 수 있습니다. 측량 영역 안은 `homography`, 밖은
정확도가 낮을 수 있는 `homography_extrapolated`로 표시됩니다.

호모그래피 좌표는 검출 프레임마다 다음 토픽으로도 발행합니다.

```bash
ros2 topic echo /camera_open/vision/fallen_location
ros2 topic echo /camera_alley/vision/fallen_location
```

메시지 타입은 `geometry_msgs/msg/PointStamped`, `frame_id`는 `map`입니다.
측량 신뢰 영역 밖의 좌표도 외삽값으로 발행하므로 이동에 사용할 때는 오차를
감안해야 합니다.

## 로봇 카메라 구조 인력 검출

AED 도착 뒤 현장 탐색에는 TurtleBot4 OAK-D용 프로필을 사용합니다.

```bash
ros2 launch aed_vision robot_vision.launch.py robot_id:=robot1
```

`robot` 모드는 파인튜닝 모델의 `fallen_person`과 COCO 모델의 `person`을 함께
검출합니다. 두 bbox가 겹치는 person은 환자로 보고 제외하며, 남은 person을
구조 인력 후보로 사용합니다. 최근 6프레임 중 3프레임 이상 후보가 있으면
다음 로봇 namespace 토픽이 `true`가 됩니다.

```bash
ros2 topic echo /robot1/vision/helper_count
ros2 topic echo /robot1/vision/helper_confirmed
```

`helper_mission_controller`는 `helper_confirmed=true`의 최신 수신값을 확인하는
즉시 제자리 회전과 반복 호출음을 중지합니다. 과거 검출이 시간 창에 남아
있더라도 현재 프레임에 사람이 없으면 `helper_confirmed`는 즉시 `false`가 됩니다.
