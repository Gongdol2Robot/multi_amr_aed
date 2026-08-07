# helper_mission

담당: 김민성

AED 도착 뒤 현장에 도움을 줄 사람이 없으면 대기 AMR이 구조 인력을 호출하고
환자 위치까지 안내하는 후속 미션입니다. 현재 알림 장치는 TurtleBot4의
`cmd_audio`에 짧은 2음 부저를 발행합니다. 블루투스 스피커를 준비한 뒤에는
`_publish_buzzer()`를 TTS publisher/client로 교체할 수 있습니다.
TurtleBot4 메시지가 없는 개발 PC에서는 노드가 종료되지 않고
`cmd_audio_fallback`에 `BEEP`/`STOP` 문자열을 발행합니다.

## 동작 흐름

1. coordinator가 `EmergencyEvent`와 AED 로봇의 `ARRIVED` 상태를 저장합니다.
2. `/aed/helper_presence`에서 같은 event의 `helper_count=0`,
   `evidence_count>=3`을 받습니다.
3. AED를 전달한 로봇을 제외하고 정상·가용·배터리 20% 이상인 로봇을 고릅니다.
4. 선택한 로봇의 `/{robot_id}/aed/guide_helper` Action에 A03 Goal을 보냅니다.
5. 로봇은 설정된 구조 인력 대기 위치로 이동해 부저를 반복합니다.
6. 3회 이상 검출되고 3m 안에 있는 구조 인력을 확인하면 환자 위치로 이동합니다.
7. 환자 위치에서 구조 인력이 1m 안에 2초 연속 확인되면 `COMPLETED`가 됩니다.

검출이 끊기거나 Nav2가 실패하거나 Goal이 취소되면 Action은 실패 사유를
반환하고 부저 및 진행 중인 Nav2 Goal을 정리합니다. 이전 mission version의
Goal도 거부합니다.

## 실행

구조 인력 대기 좌표는 시설에서 측정한 값으로 반드시 지정해야 합니다. frame을
비워두면 임의의 `(0, 0)`으로 출동하지 않고 `RECOVERY_WAIT`를 발행합니다.

```bash
ros2 launch helper_mission helper_mission.launch.py \
  robot_ids:=robot1,robot2 \
  helper_station_frame:=map \
  helper_station_x:=2.4 \
  helper_station_y:=-1.1 \
  helper_station_yaw:=1.57
```

각 로봇에서는 다음 이름이 namespace에 맞춰 해석됩니다.

- `aed/guide_helper` → `/robotN/aed/guide_helper`
- `navigate_to_pose` → `/robotN/navigate_to_pose`
- `cmd_audio` → `/robotN/cmd_audio`

coordinator는 공통 `/aed/emergency_event`, `/aed/robot_state`,
`/aed/mission_status`, `/aed/helper_presence`를 사용합니다.

## 조력자 검출 입력

Vision 노드는 `aed_interfaces/HelperPresence`를 발행해야 합니다.

```text
event_id       현재 응급 이벤트 ID
robot_id       검출 카메라가 붙은 로봇 ID
helper_count   검출된 구조 인력 수
evidence_count 연속 검출 근거 수
distance_m     로봇과 구조 인력 사이 거리
helper_pose    map 좌표계 구조 인력 위치
stamp          검출 시각
```

AED 도착 직후의 `helper_count=0` 판정은 helper 미션을 시작하고, 미션 도중의
`helper_count>0` 판정은 호출 성공 및 현장 도착 확인에 사용됩니다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `helper_call_timeout` | 30초 | 대기 위치에서 구조 인력을 기다리는 시간 |
| `minimum_evidence` | 3 | 구조 인력 확정에 필요한 검출 근거 수 |
| `call_distance_m` | 3.0m | 호출 성공 최대 거리 |
| `arrival_distance_m` | 1.0m | 현장 도착 최대 거리 |
| `arrival_hold_seconds` | 2.0초 | 도착 판정 연속 유지 시간 |
| `buzzer_frequencies` | 880, 660Hz | 임시 호출 부저음 |
