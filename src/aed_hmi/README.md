# aed_hmi

두 AMR의 상태·위치·역할 표시, 운영자 제어, 이벤트 이력 조회를 제공합니다.

관제실 화면 하나로 다음을 봅니다.

- 진행 중인 응급 상황과 경과 시간, 도착 예상 시간
- 카메라 4갈래 영상 (고정 웹캠 2대 + 로봇 OAK-D 2대). 검출이 잡히면 테두리로 알린다
- 로봇별 속도·배터리·위치·통신 상태
- 신고에서 AED 도착까지 걸린 시간과 그 이력
- 도착 예상이 실제와 얼마나 달랐나

영상 타일은 **숫자키 1~4** 로 하나만 크게 볼 수 있습니다. 같은 키나 Esc 로
4분할로 돌아옵니다. 타일을 눌러도 되고 머리글의 버튼으로도 됩니다.

## 실행

### 실제 ROS에 붙여서

HMI는 중앙 제어 launch에 포함되지 않습니다. 필요할 때만 별도 launch로
로봇 상태 발행기와 웹 백엔드를 올립니다.

```bash
cd ~/turtlebot4_ws/multi_amr_aed
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  aed_interfaces robot_state_monitor aed_hmi   # 최초 1회
source install/setup.bash
export ROS_SUPER_CLIENT=True                    # 없으면 토픽이 안 보인다

ros2 launch aed_hmi hmi_runtime.launch.py
```

다른 PC의 브라우저에서도 접속해야 할 때만 백엔드 바인딩을 엽니다.

```bash
ros2 launch aed_hmi hmi_runtime.launch.py backend_host:=0.0.0.0
```

다른 터미널에서 화면을 띄웁니다.

```bash
cd src/aed_hmi/frontend
npm install     # 최초 1회
npm run dev     # http://localhost:5173
```

백엔드 없이 `/aed/robot_state` 발행만 점검하려면 다음처럼 실행합니다.

```bash
ros2 launch aed_hmi hmi_runtime.launch.py start_backend:=false
```

HMI 지도 클릭은 `/aed/emergency_event`를 발행하며
`multi_robot_emergency`가 이를 직접 구독합니다. 따라서 실제 출동 시험에는
평소처럼 중앙 제어를 별도 터미널에서 `dispatch_enabled:=true`로 실행해야
합니다. HMI를 종료해도 중앙 제어와 주행은 계속 동작합니다.

## 필요한 것

```bash
pip install -r src/aed_hmi/requirements.txt
```

HMI 백엔드는 실제 ROS 토픽만 사용합니다. 실행 전에 ROS 환경과 빌드한
워크스페이스를 source해야 합니다.

Node는 18 이상이 필요합니다(Vite 요구). 없으면 sudo 없이 설치할 수 있습니다.

```bash
curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
nvm install 20
```

### 다른 PC 에서 처음 받았다면

```bash
git clone git@github.com:Gongdol2Robot/multi_amr_aed.git
cd multi_amr_aed

pip install -r src/aed_hmi/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  aed_interfaces robot_state_monitor aed_hmi
source install/setup.bash

# 터미널 1: 실제 ROS 상태와 HMI 백엔드
ros2 launch aed_hmi hmi_runtime.launch.py
# 터미널 2: 화면
cd src/aed_hmi/frontend && npm install && npm run dev
```

시연 영상(`docs/videos/`)도 저장소에 들어 있어 따로 받을 것이 없습니다.

ROS 에 붙일 때 `aed_interfaces` 빌드가 이 오류로 막히면,

```
canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
```

setuptools 와 packaging 의 조합 문제입니다. `pip install -U "packaging>=24"`
로 풀립니다.

## 구조

경계를 나눈 기준은 "이 파일은 무엇을 알아야 하는가"입니다.

```
backend/
├── domain/     타입과 상태 이름. ROS도 HTTP도 모른다
├── ros/        rclpy 구독과 메시지 변환. 여기만 aed_interfaces를 안다
├── store/      SQLite 스키마와 질의. SQL은 여기 밖으로 안 나간다
├── stream/     현재 상태 취합, 영상 버퍼, WebSocket 허브
├── api/        HTTP·WebSocket 라우터. 로직 없이 연결만 한다
└── context.py  위의 것들을 묶어 프로세스 하나로 만든다

frontend/src/
├── types/      backend/domain/models.py 와 1:1
├── api/        HTTP·WebSocket 클라이언트
├── hooks/      상태 구독
├── components/ layout · video · robot · mission · common
└── styles/     테마와 배치
```

`domain/`이 ROS를 모르게 둔 것이 이 구조의 핵심입니다. 메시지 변환과 계산을
ROS 실행 상태와 분리해 단위 테스트할 수 있고, 리뷰할 때도 ROS를 몰라도
읽힙니다.

## 인터페이스 사용

`aed_interfaces`의 메시지를 그대로 씁니다. uint8 상수를 사람이 읽는 이름으로
바꾸는 곳은 `domain/enums.py` 한 곳뿐이고, 프론트엔드 타입도 같은 이름을
문자열 유니온으로 갖습니다. `.msg`가 바뀌면 그 파일만 고치면 됩니다.

| 메시지 | 쓰는 곳 |
|---|---|
| `RobotState` | 로봇 카드 전체. 속도만 백엔드가 pose로 계산한다 |
| `MissionStatus` | 임무 상태 14종. 이력 표와 배너 |
| `EmergencyEvent` | 경보 배너, 검출 표시, 응답 시간 기준 시각 |
| `MissionAssignment` | 목표 좌표와 재할당 횟수. `DeliverAed` action 으로 바뀔 예정 |

## 구독하는 토픽

```
/aed/robot_state                            RobotState
/aed/mission_status                         MissionStatus
/aed/emergency_event                        EmergencyEvent
/{camera_id}/vision/emergency_event         EmergencyEvent   (vision_detector)
/{camera_id}/vision/person_count            UInt32           (vision_detector)
/{camera_id}/vision/debug/compressed        CompressedImage  (검출 표시된 영상)
/{robot_id}/oakd/rgb/preview/image_raw      Image (HMI에서 JPEG 변환)
/{robot_id}/lidar_state                    String (sensor_recovery watchdog)
/{robot_id}/fallback_state                 String (Depth/cmd_vel 전환 상태)
/emergency/eta/result                       String (JSON)    (multi_robot_emergency)
```

마지막 것만 QoS 가 다릅니다(`TRANSIENT_LOCAL`). 보내는 쪽과 맞추지 않으면
ROS 2 는 연결을 아예 안 맺고 경고도 없습니다.

`camera_id`는 `camera_open`, `camera_alley`입니다(`aed_vision/config/*.yaml`).
토픽은 환경변수로 바꿔 끼울 수 있습니다.

```bash
AED_HMI_STREAM_ROBOT1=/robot1/vision/debug/compressed python3 -m backend.main
```

로봇 쪽 검출 노드가 준비되면 이 변수만 주면 됩니다.

## 아직 안 된 것

- **로봇 OAK-D는 검출을 하지 않습니다.** 영상만 나옵니다.
- **영상은 MJPEG입니다.** 지연이 문제가 되면 WebRTC로 바꿔야 하고, 그때
  고칠 곳은 `api/video.py`와 `components/video/VideoTile.tsx`뿐입니다.

인터페이스별 구현 상태와 DB 대응은 [../../docs/db_interfaces.md](../../docs/db_interfaces.md),
키·쿼리 설계는 [../../docs/db_queries.md](../../docs/db_queries.md) 에 있습니다.
