# Multi-AMR Emergency AED Response System

서로 다른 Dock에 AED를 탑재하고 대기하는 두 대의 TurtleBot4 중 예상
이동비용이 가장 작은 로봇 한 대를 우선 출동시키는 ROS 2 프로젝트입니다.
우선 로봇에 경로 또는 네트워크 장애가 발생하면 다른 로봇으로 임무를 자동
재할당하여 단일 장애 상황에서도 AED 전달을 지속합니다.

기획 기준: [장애 복구형 시나리오 v3.0](https://app.notion.com/p/3b35af693d3581f8a95feaa09b63a4d7)

## Repository layout

```text
multi_amr_aed/
├── src/
│   ├── aed_interfaces/          # 이벤트·상태·Heartbeat·미션 메시지
│   ├── aed_vision/              # YOLO 검출 및 응급 이벤트 확정
│   ├── emergency_location_mapper/ # 카메라/구역을 지도 좌표로 변환
│   ├── mission_manager/         # 경로비용 비교, 배정·재할당·최종 실패
│   ├── robot_missions/          # Undock, Nav2 AED 전달, 도착 판정
│   ├── robot_state_monitor/     # 위치·배터리·Localization·Nav2 상태
│   ├── heartbeat/               # 관제-로봇 양방향 Heartbeat
│   ├── safety_watchdog/         # 통신 장애 시 Goal 취소와 안전 정지
│   ├── event_logger/            # 이벤트·장애·재할당 이력 저장
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
- 하나의 응급 이벤트에는 한 대의 로봇만 활성 출동합니다.
- 재할당은 증가하는 assignment version으로 중복 출동을 방지합니다.
- 소스와 설정만 Git으로 관리하고 `build`, `install`, `log`는 제외합니다.
- 학습 데이터와 대용량 모델은 저장소에 직접 커밋하지 않습니다.
- 기존 프로젝트 코드는 기능별로 검토한 뒤 해당 패키지로 이관합니다.

## Collaboration workflow

팀원은 `main` 브랜치에서 직접 개발하지 않고, 맡은 기능별 브랜치를 만들어
작업합니다.

```bash
git switch main
git pull origin main
git switch -c feature/<기능명>
```

기능이 어느 정도 완성되고 로컬 빌드·기본 동작 확인이 끝나면 원격 브랜치에
푸시하고 Pull Request를 생성합니다.

```bash
git add <변경한 파일>
git commit -m "구현 내용 요약"
git push -u origin feature/<기능명>
```

PR에는 구현 내용, 확인 방법, 아직 남은 문제를 작성합니다. 다른 팀원의 리뷰와
충돌 확인을 거친 뒤 `main`에 병합하며, 병합 후 각 팀원은 최신 `main`을 다시
받아 다음 작업 브랜치를 만듭니다.

- 하나의 브랜치에는 가능한 한 하나의 기능이나 목적만 포함합니다.
- 빌드되지 않는 코드와 개인 환경 경로는 PR에 올리지 않습니다.
- 다른 팀원의 담당 파일을 함께 수정했다면 PR 설명에 변경 이유를 남깁니다.
- 긴급한 수정이 아니라면 `main`에 직접 push하지 않습니다.

## Current migration status

- `aed_vision`: 웹캠 압축 이미지 발행과 호모그래피 좌표 변환 이관 완료
- `emergency_alert`: TurtleBot4 긴급 부저 이관 완료
- `aed_interfaces`: 응급 이벤트·로봇 상태·미션 배정 메시지 구현 완료
- `robot_missions`: typed 상태를 발행하는 단일 AED Nav2 실행기 구현 완료
- `mission_manager`: 경로비용 기반 우선 배정과 장애 재할당 골격 구현 완료
- `aed_bringup`: 실기 Nav2 안전 설정 이관 완료
- `heartbeat`, `safety_watchdog`, `event_logger`: 책임과 인터페이스 구조 정의
- 나머지 패키지: 실제 상태 수집, Watchdog, DB, HMI, 통합 launch 구현 예정

## Core scenarios

1. 정상 출동: 유효 경로 중 예상 이동비용이 가장 작은 로봇 한 대를 선택
2. 경로 장애: 지속적 Nav2 실패를 확정하고 다른 로봇으로 자동 재할당
3. 네트워크 장애: 양방향 Heartbeat timeout, 기존 로봇 정지, 대체 로봇 출동
4. 최종 실패: 두 로봇 모두 불가하면 `MISSION_FAILED`와 수동 대응 요청

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
