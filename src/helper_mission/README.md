# helper_mission

담당: 김민성

AED를 전달한 로봇이 사고 지점에서 구조 인력을 직접 찾는 후속 임무입니다.
기존처럼 다른 로봇이 구조 인력 대기 위치로 이동하지 않습니다. 도착 로봇이
제자리 회전하면서 호출음을 반복하고, 로봇의 OAK-D 영상을 처리하는
`aed_vision`이 구조 인력을 확정하면 즉시 회전과 호출음을 멈춥니다.

## 동작 흐름

1. coordinator가 `EmergencyEvent`와 AED 로봇의 `ARRIVED` 상태를 받습니다.
2. 도착한 동일 로봇의 `/{robot_id}/aed/guide_helper` Action을 시작합니다.
3. 로봇은 `/{robot_id}/cmd_vel`에 각속도만 발행해 제자리 회전합니다.
4. 구조 인력이 감지될 때까지 `/{robot_id}/cmd_audio`로 2음 호출음을 반복합니다.
5. `aed_vision`이 쓰러진 대상과 겹치지 않는 COCO `person`을 최근 6프레임 중
   3프레임 이상 검출하면 `/{robot_id}/vision/helper_confirmed=true`를 냅니다.
6. controller는 최신 true를 받는 즉시 0 속도와 오디오 정지 명령을 보냅니다.
7. 블루투스 TTS 대신 임시 상승 3음 안내 신호를 한 번 재생하고 완료합니다.

`helper_wait_timeout`의 기본값은 `0`이므로 구조 인력이 올 때까지 계속
회전·호출합니다. 취소, 예외, 노드 종료 시에도 반드시 0 속도와 오디오 정지
명령을 발행합니다. 단, `aed_vision` 메시지가 처음부터 들어오지 않거나 마지막
수신 이후 5분 동안 끊기면 카메라 장애로 판단하여 회전과 호출음을 정지합니다.

## 실행

로봇마다 OAK-D Vision 노드를 실행합니다.

```bash
ros2 launch aed_vision robot_vision.launch.py robot_id:=robot1
ros2 launch aed_vision robot_vision.launch.py robot_id:=robot2
```

중앙 coordinator와 두 로봇의 controller를 실행합니다.

```bash
ros2 launch helper_mission helper_mission.launch.py \
  robot_ids:=robot1,robot2 \
  rotation_speed_rps:=0.35 \
  helper_wait_timeout:=0.0 \
  vision_timeout_seconds:=300.0
```

각 controller의 상대 토픽은 namespace에 따라 다음처럼 해석됩니다.

- `vision/helper_confirmed` → `/robotN/vision/helper_confirmed`
- `cmd_vel` → `/robotN/cmd_vel`
- `cmd_audio` → `/robotN/cmd_audio`
- `aed/guide_helper` → `/robotN/aed/guide_helper`

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `rotation_speed_rps` | `0.35` | 제자리 회전 각속도(rad/s) |
| `control_period` | `0.1` | 회전 명령 발행 주기(초) |
| `vision_stale_seconds` | `1.0` | Vision true 신호의 최대 유효 시간 |
| `vision_timeout_seconds` | `300.0` | Vision 메시지 단절 안전 정지 시간(5분) |
| `helper_wait_timeout` | `0.0` | 탐색 제한 시간, 0이면 무제한 |
| `buzzer_period` | `1.0` | 호출음 반복 주기(초) |
| `buzzer_frequencies` | `880,660` | 임시 호출 2음(Hz) |
| `guide_frequencies` | `523,659,784` | 임시 안내 상승 3음(Hz) |

## TTS 교체 지점

블루투스 스피커가 준비되면
`HelperMissionController._publish_guide_tone()`을 TTS publisher 또는 client로
교체하면 됩니다. 구조 인력 탐색 중 반복 호출도 음성으로 바꾸려면
`_publish_call_tone()`을 같은 방식으로 교체합니다.
