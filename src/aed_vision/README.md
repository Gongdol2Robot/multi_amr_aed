# aed_vision

고정 웹캠 영상에서 쓰러진 구조 대상과 구조 보조자(`helper`)를 검출하고,
골목 카메라에서는 실제 사람 수를 이용해 통로 혼잡도까지 판단하는 ROS 2
패키지입니다.

담당: 김지훈(호모그래피·위치 검증), 이현민(구조 대상·사람 검출 및 통합)

## 카메라 구성

같은 코드를 두 노트북에 설치하고 YAML 설정만 다르게 실행합니다. 두 노트북은
같은 ROS 2 네트워크와 `ROS_DOMAIN_ID`를 사용해야 합니다.

| 카메라 | 모드 | 파인튜닝 구조 모델 | COCO person 모델 | 혼잡도 |
|---|---|---:|---:|---:|
| `camera_open` | `open` | 사용 | 미사용 | `NOT_APPLICABLE` |
| `camera_alley` | `alley` | 사용 | 사용 | `CLEAR`/`CROWDED` |

- `open`: 탁 트인 장소에서 `fallen_person`과 `helper`만 검출합니다.
- `alley`: 좁은 통로에서 구조 검출과 ROI 내부 실제 사람 수를 함께 계산합니다.
- 학습 가중치 내부 클래스명은 `helper_rc_car`지만 디버그 bbox와 상태 JSON에는
  역할 중심 이름인 `helper`를 사용합니다.

## 노드

### `vision_detector`

- 각 노트북의 USB 웹캠을 직접 읽어 같은 프로세스에서 즉시 추론
- 읽은 원본 영상은 모니터링용 JPEG 압축 토픽으로도 발행
- 두 모드 모두 파인튜닝 YOLO11n으로 `fallen_person`, `helper` 검출
- 최근 10프레임 중 6프레임 이상 검출될 때 응급상황 확정
- 확정/해제 전환 시 `aed_interfaces/EmergencyEvent` 발행
- 검출 bbox 하단 중앙점을 카메라별 호모그래피로 map 좌표 변환
- `alley` 모드에서는 COCO YOLO11n으로 ROI 내부 `person` 수 계산
- COCO가 쓰러진 대상을 person으로 중복 검출하면 bbox IoU를 이용해 인파에서 제외
- 상태 JSON, 혼잡도, 사람 수, heartbeat, JPEG 디버그 영상 발행
- 기본 설정에서는 실행한 노트북에 OpenCV 실시간 검출 창 표시

한 프레임을 처리하는 동안 새 프레임이 도착하면 큐 깊이 1의 best-effort QoS를
사용해 오래된 프레임을 쌓지 않습니다.

실행 launch는 카메라별 `vision_detector` 노드 하나만 시작합니다.

## 설치

```bash
cd ~/rokey_ws/multi_amr_aed
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r src/aed_vision/requirements.txt
colcon build --packages-select aed_interfaces aed_vision
source install/setup.bash
```

두 모델은 패키지의 `models/`에 포함되고 빌드할 때 ROS share 폴더에 함께
설치됩니다.

- `models/rescue_yolo11n.pt`: 파인튜닝 구조 검출 모델
- `models/coco_yolo11n.pt`: COCO person 검출 모델

YAML은 절대 경로 대신 다음 ROS 패키지 URI를 사용하므로 노트북마다 경로를
수정할 필요가 없습니다.

```text
package://aed_vision/models/rescue_yolo11n.pt
package://aed_vision/models/coco_yolo11n.pt
```

## 실행

탁 트인 공간 노트북:

```bash
ros2 launch aed_vision camera_vision.launch.py \
  camera:=1
```

`camera:=1`은 `camera_open` namespace와 `open_camera.yaml`을 자동 선택하며,
파인튜닝 구조 모델만 실행해 쓰러진 사람과 helper를 검출합니다. 실행한
노트북에는 `AED Vision - camera_open (open)` 결과 창이 표시됩니다.

좁은 골목 노트북:

```bash
ros2 launch aed_vision camera_vision.launch.py \
  camera:=2
```

`camera:=2`는 `camera_alley` namespace와 `alley_camera.yaml`을 자동 선택하며,
쓰러진 사람과 helper 검출에 COCO person 기반 인파 감지를 추가합니다. 실행한
노트북에는 구조 bbox, person bbox와 혼잡 ROI가 합쳐진 결과 창이 표시됩니다.

실행 전 각 YAML에서 다음 값을 현장에 맞게 수정합니다.

- `camera_device`: USB 웹캠 장치 경로. 기본값은 `/dev/video2`이며 가능하면
  재부팅 후에도 유지되는 `/dev/v4l/by-id/...` 경로 사용 권장
- `inference_device`: YOLO 추론 장치 (`"cuda:0"`은 첫 GPU, `"cpu"`는 CPU)
- `location_x`, `location_y`: 해당 고정 카메라 구조 지점의 map 좌표
- `homography_camera_id`: 카메라별 측량 설정 ID (`cam1` 또는 `cam2`)
- `homography_margin_m`: 측량 영역 경계에서 허용할 좌표 여유
- `crowd_roi`: 골목 영상에서 AMR이 통과해야 하는 영역
- `crowded_person_threshold`: `CROWDED`로 판단할 최소 사람 수
- `show_window`: 해당 노트북에 OpenCV 결과 창을 표시할지 여부

## 토픽

`<camera_id>`는 `camera_open` 또는 `camera_alley`입니다.

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/<camera_id>/image_raw/compressed` | `sensor_msgs/CompressedImage` | 로컬 웹캠 JPEG |
| `/<camera_id>/vision/emergency_event` | `aed_interfaces/EmergencyEvent` | 확정 또는 해제된 구조 이벤트 |
| `/<camera_id>/vision/status` | `std_msgs/String` | 전체 검출 상태 JSON |
| `/<camera_id>/vision/crowd_level` | `std_msgs/String` | `NOT_APPLICABLE`, `0`, `1`, `2`, `3` (3은 3명 이상) |
| `/<camera_id>/vision/person_count` | `std_msgs/UInt32` | 골목 ROI 내 유효 person 수 |
| `/<camera_id>/vision/fallen_location` | `geometry_msgs/PointStamped` | 호모그래피로 계산한 구조 대상 map 좌표 |
| `/<camera_id>/vision/heartbeat` | `aed_interfaces/Heartbeat` | 초당 노드 생존 신호 |
| `/<camera_id>/vision/debug/compressed` | `sensor_msgs/CompressedImage` | bbox와 ROI가 표시된 JPEG |

`status` JSON 예시:

```json
{
  "camera_id": "camera_alley",
  "zone_id": "alley_zone",
  "mode": "alley",
  "fallen_detected": true,
  "fallen_confirmed": true,
  "fallen_count": 1,
  "fallen_max_confidence": 0.91,
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
행렬을 사용합니다. 가장 confidence가 높은 `fallen_person` bbox의 하단 중앙점을
바닥 접점으로 보고 `map` 좌표로 변환하여 `EmergencyEvent.location`에 넣습니다.

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
