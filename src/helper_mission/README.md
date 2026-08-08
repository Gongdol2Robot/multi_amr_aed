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
4. 구조 인력이 감지될 때까지 PC 기본 출력(블루투스 스피커)으로 CC0 호출
   경고음을 끊지 않고 연속 반복합니다.
5. `aed_vision`이 쓰러진 대상과 겹치지 않는 COCO `person`을 최근 6프레임 중
   3프레임 이상 검출하면 `/{robot_id}/vision/helper_confirmed=true`를 냅니다.
6. controller는 최신 true를 받는 즉시 0 속도와 오디오 정지 명령을 보냅니다.
7. 블루투스 스피커로 `조력자를 확인했습니다. AED를 인계한 후 복귀합니다.`
   TTS를 재생하고 5초 동안 정지 상태로 인계를 기다립니다.
8. `HELPER_ARRIVED`를 발행하면 중앙 제어 노드가 출동 직전 위치로 복귀
   임무를 전송합니다.

`helper_wait_timeout`의 기본값은 `0`이므로 구조 인력이 올 때까지 계속
회전·호출합니다. 취소, 예외, 노드 종료 시에도 반드시 0 속도와 오디오 정지
명령을 발행합니다. 정지 속도 명령은 기본 3회 반복해 단일 메시지 유실 위험을
줄입니다. `aed_vision` 메시지가 처음부터 들어오지 않거나 마지막 수신 이후
5분 동안 끊기면 카메라 장애로 판단하여 회전과 호출음을 정지합니다.

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
- 시스템 오디오 → OS 기본 출력 장치(권장: 블루투스 스피커)
- `aed/guide_helper` → `/robotN/aed/guide_helper`

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `rotation_speed_rps` | `0.35` | 제자리 회전 각속도(rad/s) |
| `control_period` | `0.1` | 회전 명령 발행 주기(초) |
| `stop_command_repeats` | `3` | 종료 시 0속도 명령 반복 횟수 |
| `vision_stale_seconds` | `1.0` | Vision true 신호의 최대 유효 시간 |
| `vision_timeout_seconds` | `300.0` | Vision 메시지 단절 안전 정지 시간(5분) |
| `helper_wait_timeout` | `0.0` | 탐색 제한 시간, 0이면 무제한 |
| `buzzer_period` | `2.2` | 약 2초인 호출음의 반복 주기(초) |
| `buzzer_frequencies` | `880,660` | 임시 호출 2음(Hz) |
| `guide_frequencies` | `523,659,784` | 임시 안내 상승 3음(Hz) |
| `audio_backend` | `system` | PC 스피커 사용, `create3`이면 기존 본체 부저 |
| `audio_player` | `auto` | `paplay`, `pw-play`, `aplay` 자동 선택 |
| `audio_device` | 빈 문자열 | 빈 값이면 OS 기본 출력, 필요 시 장치 지정 |
| `call_audio_file` | 내장 CC0 WAV | 노드 직접 실행 시 별도 호출음 파일 지정 |
| `handoff_wait_seconds` | `5.0` | TTS 재생 후 정지 상태로 인계를 기다리는 시간 |
| `handoff_audio_file` | 내장 한국어 WAV | 노드 직접 실행 시 별도 안내 TTS 지정 |

## TTS 교체 지점

호출 경고음은 `emergency_alert/assets/cc0_warning_alarm.wav`(CC0), 인계 안내는
`emergency_alert/assets/helper_confirmed_return_ko.wav`를 사용합니다. 각각
`call_audio_file`, `handoff_audio_file`로 교체할 수 있습니다.
