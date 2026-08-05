# Multi-AMR Emergency AED Response System

두 대의 TurtleBot4가 평시 순찰·서비스 임무를 수행하고, 응급상황에서는
AED 운반과 구조 인력 길안내 역할을 동적으로 분담하는 ROS 2 프로젝트입니다.

## Repository layout

```text
multi_amr_aed/
├── src/
│   ├── aed_interfaces/          # 메시지, 서비스, 액션 정의
│   ├── aed_vision/              # YOLO 검출 및 응급 이벤트 확정
│   ├── mission_manager/         # 역할 배정과 미션 상태 머신
│   ├── robot_missions/          # 순찰, 청소, AED, 길안내
│   ├── robot_state_monitor/     # 위치, 배터리, 통신, Nav2 상태
│   ├── emergency_alert/         # 부저 및 음성 알림
│   ├── aed_hmi/                 # 웹 관제 및 이벤트 이력
│   └── aed_bringup/             # 멀티 로봇 launch와 설정
├── config/
├── maps/
├── models/
├── tools/
├── docs/
└── tests/
```

## Development policy

- 로봇은 ROS 2 namespace와 robot ID로 구분합니다.
- 소스와 설정만 Git으로 관리하고 `build`, `install`, `log`는 제외합니다.
- 학습 데이터와 대용량 모델은 저장소에 직접 커밋하지 않습니다.
- 기존 프로젝트 코드는 기능별로 검토한 뒤 해당 패키지로 이관합니다.

## Current migration status

- `aed_vision`: 웹캠 압축 이미지 발행과 호모그래피 좌표 변환 이관 완료
- `emergency_alert`: TurtleBot4 긴급 부저 이관 완료
- `aed_interfaces`: 응급 이벤트·로봇 상태·미션 배정 메시지 구현 완료
- `robot_missions`: namespace 기반 Nav2 미션 실행기 구현 완료
- `mission_manager`: 두 로봇 거리 기반 AED·길안내 동적 배정 구현 완료
- `aed_bringup`: 실기 Nav2 안전 설정 이관 완료
- 나머지 패키지: 상태 수집, HMI, 통합 launch를 순차 구현

## Workspace setup

이 저장소 자체가 `src/`를 포함한 독립적인 colcon workspace입니다. 기존 경로와
단축어가 꼬이지 않도록 다음 위치를 기본 설치 경로로 사용합니다.

```text
~/rokey_ws/
└── multi_amr_aed/       # Git 저장소이자 독립 colcon workspace
    ├── src/
    ├── tools/
    ├── build/
    ├── install/
    └── log/
```

> **주의:** `~/rokey_ws/src/multi_amr_aed`에는 clone하지 마세요. 이 저장소
> 내부에 이미 `src/`가 있어 workspace가 중첩됩니다.

```bash
mkdir -p ~/rokey_ws
cd ~/rokey_ws
git clone https://github.com/Gongdol2Robot/multi_amr_aed.git
cd multi_amr_aed

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

단축 명령을 사용하려면 다음 한 줄을 등록합니다.

```bash
echo 'source ~/rokey_ws/multi_amr_aed/tools/aliases.sh' >> ~/.bashrc
source ~/.bashrc
aedenv
```

## Build

```bash
cd ~/rokey_ws/multi_amr_aed
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

환경별 지도, 카메라 보정값, YOLO 모델은 저장소 기본값으로 간주하지 않습니다.
현장 측정과 모델 검증 후 별도로 설정해야 합니다.
