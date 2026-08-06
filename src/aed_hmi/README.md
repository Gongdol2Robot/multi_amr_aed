# aed_hmi

두 AMR의 상태·위치·역할 표시, 운영자 제어, 이벤트 이력 조회를 제공합니다.

관제실 화면 하나로 다음을 봅니다.

- 진행 중인 응급 상황과 경과 시간
- 카메라 4갈래 영상 (고정 웹캠 2대 + 로봇 OAK-D 2대)
- 로봇별 속도·배터리·위치·통신 상태
- 신고에서 AED 도착까지 걸린 시간과 그 이력

## 실행

### 실제 ROS에 붙여서

```bash
cd ~/rokey_ws/multi_amr_aed
source /opt/ros/humble/setup.bash
colcon build --packages-select aed_interfaces   # 최초 1회
source install/setup.bash
export ROS_SUPER_CLIENT=True                    # 없으면 토픽이 안 보인다

cd src/aed_hmi
python3 -m backend.main
```

다른 터미널에서 화면을 띄웁니다.

```bash
cd src/aed_hmi/frontend
npm install     # 최초 1회
npm run dev     # http://localhost:5173
```

### 장비 없이 (화면 개발·시연용)

```bash
python3 -m backend.main --mock
```

신고 → 배정 → 이동 → 도착 → 복귀를 반복 재생합니다. 3번에 1번은 경로
장애를 만들어 재할당 화면도 나옵니다. 로봇을 다른 팀원이 쓰고 있어도
화면 작업이 막히지 않게 하려고 넣었습니다.

## 필요한 것

```bash
pip install fastapi "uvicorn[standard]" websockets
```

Node는 18 이상이 필요합니다(Vite 요구). 없으면 sudo 없이 설치할 수 있습니다.

```bash
curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
nvm install 20
```

## 구조

경계를 나눈 기준은 "이 파일은 무엇을 알아야 하는가"입니다.

```
backend/
├── domain/     타입과 상태 이름. ROS도 HTTP도 모른다
├── ros/        rclpy 구독과 메시지 변환. 여기만 aed_interfaces를 안다
├── store/      SQLite 스키마와 질의. SQL은 여기 밖으로 안 나간다
├── stream/     현재 상태 취합, 영상 버퍼, WebSocket 허브
├── api/        HTTP·WebSocket 라우터. 로직 없이 연결만 한다
├── mock/       장비 없이 도는 시나리오
└── context.py  위의 것들을 묶어 프로세스 하나로 만든다

frontend/src/
├── types/      backend/domain/models.py 와 1:1
├── api/        HTTP·WebSocket 클라이언트
├── hooks/      상태 구독
├── components/ layout · video · robot · mission · common
└── styles/     테마와 배치
```

`domain/`이 ROS를 모르게 둔 것이 이 구조의 핵심입니다. 그래야 로봇 없이
전체 로직을 시험할 수 있고, 리뷰할 때도 ROS를 몰라도 읽힙니다.

## 인터페이스 사용

`aed_interfaces`의 메시지를 그대로 씁니다. uint8 상수를 사람이 읽는 이름으로
바꾸는 곳은 `domain/enums.py` 한 곳뿐이고, 프론트엔드 타입도 같은 이름을
문자열 유니온으로 갖습니다. `.msg`가 바뀌면 그 파일만 고치면 됩니다.

| 메시지 | 쓰는 곳 |
|---|---|
| `RobotState` | 로봇 카드 전체. 속도만 백엔드가 pose로 계산한다 |
| `MissionStatus` | 임무 상태 14종. 이력 표와 배너 |
| `EmergencyEvent` | 경보 배너, 검출 표시, 응답 시간 기준 시각 |
| `MissionAssignment` | 목표 좌표와 재할당 횟수 |

## 구독하는 토픽

```
/aed/robot_state                            RobotState
/aed/mission_status                         MissionStatus
/aed/emergency_event                        EmergencyEvent
/{camera_id}/vision/emergency_event         EmergencyEvent   (vision_detector)
/{camera_id}/vision/person_count            UInt32           (vision_detector)
/{camera_id}/vision/debug/compressed        CompressedImage  (검출 표시된 영상)
/{robot_id}/oakd/rgb/image_raw/compressed   CompressedImage
```

`camera_id`는 `camera_open`, `camera_alley`입니다(`aed_vision/config/*.yaml`).
토픽은 환경변수로 바꿔 끼울 수 있습니다.

```bash
AED_HMI_STREAM_ROBOT1=/robot1/vision/debug/compressed python3 -m backend.main
```

로봇 쪽 검출 노드가 준비되면 이 변수만 주면 됩니다.

## 아직 안 된 것

- **검출이 출동으로 이어지지 않습니다.** `vision_detector`는
  `/{camera_id}/vision/emergency_event`로 내는데 `mission_manager`는
  `/aed/emergency_event`를 구독합니다. 둘을 잇는 노드가 필요합니다.
- **로봇 OAK-D는 검출을 하지 않습니다.** 영상만 나옵니다.
- **영상은 MJPEG입니다.** 지연이 문제가 되면 WebRTC로 바꿔야 하고, 그때
  고칠 곳은 `api/video.py`와 `components/video/VideoTile.tsx`뿐입니다.
