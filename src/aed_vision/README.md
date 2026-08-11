# aed_vision

고정 USB 웹캠과 TurtleBot4 OAK-D 영상에서 쓰러진 구조 대상과 구조
보조자(`helper`)를 검출하고, 골목 카메라에서는 실제 사람 수를 이용해 통로
혼잡도까지 판단하는 ROS 2 패키지입니다.

담당: 김지훈(호모그래피·위치 검증), 이현민(구조 대상·사람 검출 및 통합)

## 카메라 구성

같은 코드를 두 노트북에 설치하고 YAML 설정만 다르게 실행합니다. 두 노트북은
같은 ROS 2 네트워크와 `ROS_DOMAIN_ID`를 사용해야 합니다.

| 카메라 | 모드 | 기본 검출 | 선택 검출 | 혼잡도 |
|---|---|---|---|---|
| `camera_open` | `open` | 목각인형 파인튜닝 | 실제 사람 Pose | `NOT_APPLICABLE` |
| `camera_alley` | `alley` | 목각인형 파인튜닝 | 실제 사람 Pose | `CLEAR`~`BLOCKED` |
| `robot1`, `robot2` | `robot` | 목각인형·조력자 파인튜닝 | 실제 사람 Pose | `NOT_APPLICABLE` |

- `open`: 탁 트인 장소에서 쓰러진 목각인형과 조력자를 검출합니다.
- `alley`: 좁은 통로에서 구조 검출과 ROI 내부 실제 사람 수를 함께 계산합니다.
- `robot`: OAK-D 영상을 받아 환자 가까이에 있는 조력자를 확인합니다.
- 현재 배포된 구조 모델의 클래스명은 `mannequin`, `helping_person`이며 노드
  시작 시 이 순서가 맞는지 검사합니다.

## 노드

### `vision_detector`

- 각 노트북의 USB 웹캠을 직접 읽어 같은 프로세스에서 즉시 추론
- 읽은 원본 영상은 모니터링용 JPEG 압축 토픽으로도 발행
- 기본 `mannequin_detect` backend는 파인튜닝 YOLO로 목각인형을 찾고,
  HOG+SVM으로 최종 자세를 판정합니다. 운영 기본값은 객체별 Pose 추가 추론을
  끄며, SVM이 없을 때는 bbox 비율로 낙상 판정을 보완합니다.
- 선택 `person_pose` backend는 YOLO11n-Pose의 실제 사람 관절과 bbox로 자세 판정
- 동일한 낙상 bbox의 위치와 크기가 1초 이상 안정적일 때 응급상황 확정
- 확정 뒤 2초 동안 낙상 후보가 없을 때만 응급상황 해제
- 확정/해제 전환 시 `aed_interfaces/EmergencyEvent` 발행
- 검출 bbox 하단 중심점을 카메라별 호모그래피로 map 좌표 변환
- `alley` 모드에서는 COCO YOLO11n으로 ROI 내부 `person` 수 계산
- COCO가 쓰러진 대상을 person으로 중복 검출하면 bbox IoU를 이용해 인파에서 제외
- 상태 JSON, 혼잡도, 사람 수, heartbeat, JPEG 디버그 영상 발행
- 기본 설정에서는 실행한 노트북에 OpenCV 실시간 검출 창 표시

한 프레임을 처리하는 동안 새 프레임이 도착하면 큐 깊이 1의 best-effort QoS를
사용해 오래된 프레임을 쌓지 않습니다.

실행 launch는 카메라별 `vision_detector` 노드 하나만 시작합니다.

### 검출 confidence와 시간 확정 기준

고정 카메라의 구조 대상 confidence 임계값인 `rescue_conf`는 `0.60`입니다.
로봇 카메라는 근거리 현장 탐색 특성에 맞춰 `0.50`으로 덮어씁니다.
응급상황에서는 오검출보다 쓰러진 사람을 놓치는 미검출의 비용이 더 크므로,
1차 YOLO 검출에서는 재현율(recall)을 우선해 후보를 넓게 받도록 설정했습니다.

낮은 임계값에서 발생할 수 있는 순간적인 오검출을 곧바로 응급상황으로 처리하지는
않습니다. 고정 카메라는 각 프레임에서 confidence가 `0.60` 이상인
`mannequin`을 먼저 찾고 자세 판정을 통과한 대상만 낙상 후보로 받습니다.
동일한 낙상 bbox가 화면 대각선 기준 중심 이동률 2.5% 이하, 면적 변화율 25%
이하인 상태로 1초 이상 유지될 때 응급상황을 확정합니다. 다른 bbox로 바뀌거나
움직임 기준을 넘으면 1초 타이머를 다시 시작합니다. 확정 뒤에는 순간 가림으로
바로 취소되지 않도록 실제 시간 기준 2초 연속 미검출 뒤에만 `CANCELED`로
전환합니다.

```text
고정 카메라 YOLO confidence >= 0.60
        ↓ 후보 검출 — recall 우선
동일 bbox 위치·크기 안정 상태 1초 유지
        ↓ 시간·움직임 검증 — 순간 오검출 억제
EmergencyEvent.CONFIRMED
        ↓ 2초 연속 미검출
EmergencyEvent.CANCELED
```

따라서 confidence 하나만으로 응급상황을 확정하는 구조가 아니라 동일 대상의
정지 지속 시간으로 신뢰도를 보완하는 구조입니다. 현재 값은 현장 데이터로 최적화가 끝난
수치가 아니므로
최종 배포 전에는 실제 카메라 위치·조명·거리에서 precision, recall, 오검출률과
미검출률을 측정해 confidence와 정지 판정 파라미터를 함께 조정해야 합니다.

## 설치

```bash
cd ~/rokey_ws/multi_amr_aed
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r src/aed_vision/requirements.txt
colcon build --packages-select aed_interfaces aed_vision
source install/setup.bash
```

네 모델 파일은 패키지의 `models/`에 포함되고 빌드할 때 ROS share 폴더에 함께
설치됩니다.

- `models/rescue2_yolo11n.pt`: 파인튜닝 구조 검출 모델
- `models/coco_yolo11n.pt`: COCO person 검출 모델
- `models/yolo11n-pose.pt`: 실제 사람 17관절 Pose 모델
- `models/mannequin_posture_svm.xml`: 목각인형 crop의 낙상 자세 분류 모델

YAML은 절대 경로 대신 다음 ROS 패키지 URI를 사용하므로 노트북마다 경로를
수정할 필요가 없습니다.

```text
package://aed_vision/models/rescue2_yolo11n.pt
package://aed_vision/models/coco_yolo11n.pt
package://aed_vision/models/yolo11n-pose.pt
package://aed_vision/models/mannequin_posture_svm.xml
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

설정은 공통·backend·카메라 역할로 분리합니다. `base_camera.yaml`은 공통
영상·시간 상태·출력 설정, `mannequin_backend.yaml`과
`person_pose_backend.yaml`은 모델별 설정, 나머지 카메라 YAML은 설치 위치와
입력 방식을 담당합니다. 적용 순서는 `Python 안전 기본값 → base_camera.yaml
→ backend YAML → 카메라별 YAML → launch override`입니다.

운영 `camera_vision.launch.py`는 인자를 생략하면 helping RC카를 검출하는
`mannequin_backend.yaml`을 기본으로 사용합니다. 필요할 때만
`backend:=person_pose`를 명시해 실제 사람 자세 경로로 바꿀 수 있습니다.
두 backend의 역할은 다음과 같습니다.

- `person_pose`: 실제 사람의 bbox·17관절을 검출하고 종횡비와 몸통 각도로
  `STANDING`, `SITTING`, `FALLEN` 판정
- `mannequin_detect`(기본): `rescue2_yolo11n.pt`로 `mannequin`과
  `helping_person`을 검출하고, mannequin crop은 HOG+SVM으로 자세를 우선
  판정합니다. SVM을 설정하지 않은 경우 Pose와 bbox 비율이 fallback입니다.

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
- `detection_backend`: `mannequin_detect`(기본) 또는 `person_pose`
- `pose_weights`: 실제 사람용 Pose 가중치
- `person_conf`, `pose_keypoint_conf`, `pose_min_keypoints`,
  `pose_min_box_area`: Pose 사람·관절 품질 필터
- `rescue_conf`: 구조 대상 YOLO의 1차 후보 confidence 임계값. 고정 카메라
  `0.60`, 로봇 카메라 `0.50`
- `fall_stationary_seconds`: 동일 bbox가 안정적으로 유지돼야 하는 시간. `1.0`
- `fall_max_center_motion_ratio`: 화면 대각선 대비 허용 중심 이동률. `0.025`
- `fall_max_size_change_ratio`: 기준 bbox 대비 허용 면적 변화율. YOLO bbox
  jitter를 흡수하도록 `0.25`
- `fall_track_match_iou`: 동일 대상으로 연결할 최소 bbox IoU. `0.30`
- `fall_detection_gap_tolerance_seconds`: 추적 중 허용할 짧은 미검출 간격. `0.25`
- `cancellation_timeout_seconds`: 확정 뒤 해제까지 필요한 연속 미검출 시간. `2.0`
- `run_pose_for_mannequin`: SVM 판정 뒤 디버그 골격용 Pose를 추가 실행할지 여부.
  운영 기본값은 중복 추론을 막는 `false`

CPU에서 `robot_approach.mp4`의 동일한 6개 프레임을 비교했을 때 warm-up 제외
평균은 Pose 활성 `146.7 ms`(6.8 FPS), 비활성 `63.2 ms`(15.8 FPS)였습니다.
현재 운영 설정은 더 빠른 Pose 비활성 경로를 사용합니다. 두 설정의 프레임별
낙상 개수는 같았으며, 관절 디버깅이 필요할 때만 이 옵션을 `true`로 바꿉니다.
측정값은 현재 개발 노트북의 참고치이며 배포 장치 성능을 보장하지 않습니다.
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
| `/<camera_id>/vision/person_count` | `std_msgs/UInt32` | ROI 안에서 환자를 제외한 주변 person 수. 해당 경로 비활성 시 0 |
| `/<camera_id>/vision/detection_summary` | `aed_interfaces/DetectionSummary` | 프레임별 구조화된 검출 요약 |
| `/<camera_id>/vision/fallen_location` | `geometry_msgs/PointStamped` | 호모그래피로 계산한 구조 대상 map 좌표 |
| `/<camera_id>/vision/heartbeat` | `aed_interfaces/Heartbeat` | 초당 노드 생존 신호 |
| `/<camera_id>/vision/debug/compressed` | `sensor_msgs/CompressedImage` | bbox와 ROI가 표시된 JPEG |

로봇 OAK-D 모드는 로봇 oakd 노드가 image_transport로 발행하는 704x704 JPEG
스트림을 구독합니다. 운영 입력은 아래 토픽으로 고정합니다.

```text
/robotN/oakd/rgb/image_raw/compressed  (sensor_msgs/CompressedImage)
```

노트북에서 JPEG을 디코딩한 뒤 `robot_camera.yaml`에 설정된 추론
파이프라인에 입력합니다. HMI는 이 입력을 직접 구독하지 않고 중앙 비전 노드가
발행한 `/robotN/vision/debug/compressed`만 표시합니다. 로봇 비전의 backend,
confidence와 조력자 판정 기준은 launch 인자가
덮어쓰지 않으며 이 YAML을 단일 기준으로 사용합니다.
라즈베리파이에는 모델이나 추가 추론 프로세스를 설치하지 않습니다.

비압축 `preview/image_raw`(320x320 bgr8, 7fps 기준 약 17Mbps)는 로봇 WiFi
실효 대역폭(~6Mbps)을 초과해 프레임 대부분이 유실되고 로봇 핑이 수백 ms로
튀므로 운영 입력으로 쓰지 않습니다. 압축 스트림은 지연 발행(lazy publishing)
이라 구독자가 붙을 때만 로봇이 인코딩과 전송을 시작하며, 실측 기준 프레임당
약 89KB, 7fps에 약 5Mbps입니다. JPEG 인코딩은 OAK-D 장치 내부에서 수행되어
(`rgb.i_low_bandwidth`) 라즈베리파이 CPU를 쓰지 않습니다.

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
  "detection_backend": "mannequin_detect",
  "fallen_detected": true,
  "fallen_confirmed": true,
  "fallen_count": 1,
  "fallen_max_confidence": 0.91,
  "posture": "FALLEN",
  "pose_aspect_ratio": 1.91,
  "pose_torso_angle_deg": -1.0,
  "pose_visible_keypoints": 0,
  "helper_count": 0,
  "helper_confirmed": false,
  "helper_confirmation_hits": 0,
  "person_count": 3,
  "crowd_level": 3,
  "crowd_observed_level": 3,
  "crowd_time_multiplier": null,
  "crowd_traversable": false,
  "confirmation_hits": 7,
  "fallen_stationary_duration_s": 1.08,
  "fallen_center_motion_ratio": 0.0042,
  "fallen_size_change_ratio": 0.031,
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
모델이 최종 낙상으로 판정한 `mannequin` bbox와 COCO YOLO11n의 `person`
bbox가 겹치면 동일한
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
행렬을 사용합니다. 가장 confidence가 높은 낙상 후보 bbox의 하단 중심점을
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
측량 신뢰 영역 밖의 좌표도 진단용 `fallen_location`에는 외삽값으로
발행합니다. 다만 `EmergencyEvent.location_valid=false`와
`location_source=homography_extrapolated`를 넣으며 Mission Manager는 이
이벤트를 자동 이동 목표로 사용하지 않습니다.

## 로봇 카메라 구조 인력 검출

robot1·robot2는 배정과 추론 지연이 서로 영향을 주지 않도록 각각 별도
`vision_detector` 프로세스로 실행합니다.

```bash
ros2 launch aed_vision robot_vision.launch.py \
  robot_id:=robot1

ros2 launch aed_vision robot_vision.launch.py \
  robot_id:=robot2
```

두 프로세스는 모델도 각각 로드하므로 GPU 메모리는 더 사용하지만, 한 로봇의
긴 추론이 다른 로봇 콜백을 직렬로 막지 않습니다.

운영용 로봇 Vision은 실행 직후 OAK-D 영상 토픽을 구독하지 않습니다.
`/<robot_id>/mission_assignment`에서 해당 로봇의 유효한 배정을 받은 순간에만
`/<robot_id>/oakd/rgb/image_raw/compressed` 구독을 생성하고 추론을 시작합니다.
배정 전에는 Vision heartbeat만 발행합니다. 배송 도착 뒤에는 조력자 탐색을
위해 추론을 유지하고, 조력자 탐색 종료·실패·취소 또는 복귀 도착 시 이미지
구독을 제거합니다. 새 배정을 받으면 다시 구독합니다.

```text
/robot1/mission_assignment 수신
        ↓
/robot1/oakd/rgb/image_raw/compressed 구독 생성
        ↓
vision_detector 추론 시작
```

`robot_camera_model_test.launch.py`는 배정 게이트를 사용하지 않으므로 모델을
직접 확인할 때는 기존처럼 즉시 영상 구독과 추론을 시작합니다.

로봇 OAK-D 시점에서 현재 모델의 bbox와 자세 판정을 직접 확인할 때는 운영
노드 대신 다음 테스트 launch를 실행합니다.

```bash
ros2 launch aed_vision robot_camera_model_test.launch.py robot_id:=robot2
```

로컬 창을 띄울 수 없는 서버에서는 창을 끄고 디버그 영상 토픽을 확인합니다.

```bash
ros2 launch aed_vision robot_camera_model_test.launch.py \
  robot_id:=robot2 show_window:=false
ros2 run rqt_image_view rqt_image_view \
  /robot2_test/vision/debug/compressed
```

기본 입력은 `/robot2/oakd/rgb/image_raw/compressed`이고, 다른 카메라 토픽을
시험하려면 `image_topic:=/원하는/토픽`으로 바꿀 수 있습니다. 토픽 이름이
`/compressed`로 끝나면 CompressedImage로, 아니면 raw Image로 구독합니다.
테스트 결과 토픽은 `/robot2_test/vision/*`로 분리되어 운영 결과와 충돌하지
않습니다.

고정 USB 웹캠의 `camera_vision.launch.py`와 로봇 OAK-D의
`robot_vision.launch.py`는 모두 backend 인자를 생략하면 같은
`rescue2_yolo11n.pt`를 사용합니다. 이 모델의 `mannequin`은 낙상 대상,
`helping_person`은 운영 시 helping RC카를 뜻합니다.

RC카만 단독으로 보이면 임무를 끝내지 않습니다. `helping_person` bbox 하단
중심이 **같은 처리 프레임**의 낙상 환자 bbox 중심에서 화면 대각선 길이의
30% 이내일 때만 helper 후보로 남깁니다. 최근 6프레임 중 3프레임 이상 이
동시 조건을 통과하고 현재 프레임에도 두 대상이 함께 있어야 다음 로봇
namespace 토픽이 `true`가 됩니다.

```bash
ros2 topic echo /robot1/vision/helper_count
ros2 topic echo /robot1/vision/helper_confirmed
```

`helper_mission_controller`는 `helper_confirmed=true`의 최신 수신값을 확인하는
즉시 제자리 회전과 반복 호출음을 중지합니다. 과거 검출이 시간 창에 남아
있더라도 현재 프레임에 사람이 없으면 `helper_confirmed`는 즉시 `false`가 됩니다.

`backend:=person_pose`는 실제 사람을 환자와 사람 조력자로 인식하는 선택
경로이며 rescue2의 helping RC카 class는 사용하지 않습니다. 로봇 OAK-D에서
운영 토픽과 분리해 비교하려면 테스트 launch를 사용합니다.

```bash
ros2 launch aed_vision robot_camera_model_test.launch.py \
  robot_id:=robot2 backend:=person_pose
```

## 테스트

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/aed_vision:$PYTHONPATH \
  python3 -m pytest -q src/aed_vision/test
```

저장소 모델을 CPU로 직접 로드하는 느린 스모크 테스트는 명시적으로 켭니다.

```bash
AED_VISION_MODEL_TESTS=1 PYTHONPATH=src/aed_vision:$PYTHONPATH \
  python3 -m pytest -q -m model src/aed_vision/test/test_model_smoke.py
```
