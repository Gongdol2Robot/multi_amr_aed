# emergency_alert

TurtleBot4 Create3 스피커로 AED 출동·도착·중단 경보음을 재생합니다. 주행 제어와
경보 출력을 분리한 상태 구독형 노드를 기본으로 사용하며, 기존 결합형 실행기도
독립 운용이 필요한 경우를 위해 유지합니다.

## 권장 구성: 상태 구독형 경보

`mission_status_alert`는 `/aed/mission_status`만 구독하며 Nav2나 Undock Goal을
보내지 않습니다. 따라서 기존 `robot_missions/mission_executor`와 함께 실행해도
중복 주행 Goal이 발생하지 않습니다.

```text
MissionStatus.ASSIGNED/DISPATCHING  기존 경보 정지
MissionStatus.EN_ROUTE             출동 경보 반복 시작
MissionStatus.ARRIVED/COMPLETED     출동 경보 정지 + 도착음 1회
CANCELED/BLOCKED/NETWORK_LOST/
NAVIGATION_ERROR                    출동 경보 정지 + 중단음 1회
```

같은 상태의 중복 메시지와 과거 assignment version은 무시합니다. 기본적으로
mission ID가 `-aed`로 끝나는 임무만 처리하므로 helper 임무의 `COMPLETED`가 AED
도착음을 다시 재생하지 않습니다. 종료 상태가 유실되더라도 경보가 무한히
재생되지 않도록 기본 600초의 `maximum_alarm_duration` 제한도 적용됩니다.

```bash
ros2 launch emergency_alert multi_robot_status_alert.launch.py \
  robot_ids:=robot1,robot2
```

MissionStatus는 현재 volatile 토픽이므로 이 launch를 AED 출동 실행기보다 먼저
시작해야 합니다. 이미 `EN_ROUTE`가 발행된 뒤 경보 노드를 켜면 과거 상태를
재수신할 수 없으며, 향후 통합 bringup에서도 경보 노드를 먼저 기동해야 합니다.

로봇별로 직접 실행할 수도 있습니다.

```bash
ros2 run emergency_alert mission_status_alert --ros-args \
  -r __ns:=/robot1 -p robot_id:=robot1
```

이 구성에서는 각 로봇의 주행 실행기를 별도로 실행합니다.

```bash
ros2 run robot_missions mission_executor --ros-args \
  -r __ns:=/robot1 -p robot_id:=robot1
```

## 결합형 실행기

`alert_mission_executor`는 `MissionAssignment`를 받아 다음 작업을 모두 수행합니다.

1. `Undock` Action 전송
2. 출동 경보 시작
3. `NavigateToPose` Goal 전송
4. 도착·취소·실패에 따른 MissionStatus와 종료음 발행

```bash
ros2 launch emergency_alert multi_robot_alert.launch.py \
  robot_ids:=robot1,robot2
```

> 결합형 실행기는 Nav2 Goal까지 전송합니다. 같은 로봇에서
> `robot_missions/mission_executor`와 동시에 실행하지 마세요.
> 결합형 실행기 자체가 경보음도 재생하므로 `mission_status_alert`와도 동시에
> 실행하지 마세요. 같은 음 패턴이 두 publisher에서 중복 발행됩니다.

## 경보 패턴

| 상황 | 기본 패턴 | 재생 방식 |
|---|---|---|
| 출동 중 | `1000 → 440 Hz` | 각 0.25초, 0.8초마다 반복 |
| 도착 | `523 → 659 → 784 → 1047 Hz` | 각 0.2초, 1회 |
| 장애·취소 | `880 → 660 → 440 → 220 Hz` | 각 0.2초, 1회 |

주파수, 음 길이와 반복 주기는 ROS 파라미터로 변경할 수 있습니다.
`alarm_period`는 출동 음 패턴 전체 길이보다 짧게 설정할 수 없습니다. 너무 짧으면
새 메시지가 Create3 오디오 큐를 계속 교체해 뒤쪽 음이 재생되지 않기 때문입니다.

```bash
ros2 run emergency_alert mission_status_alert --ros-args \
  -r __ns:=/robot1 -p robot_id:=robot1 \
  -p alarm_period:=0.8 \
  -p travel_note_duration:=0.25 \
  -p travel_frequencies:="[1000, 440]" \
  -p terminal_note_duration:=0.2 \
  -p arrival_frequencies:="[523, 659, 784, 1047]" \
  -p interrupted_frequencies:="[880, 660, 440, 220]"
```

기본 상대 토픽은 namespace에 따라 다음처럼 해석됩니다.

- `cmd_audio` → `/robotN/cmd_audio`
- `mission_assignment` → `/robotN/mission_assignment` (결합형 전용)
- `undock` → `/robotN/undock` (결합형 전용)
- `navigate_to_pose` → `/robotN/navigate_to_pose` (결합형 전용)
- MissionStatus 입력·출력 → `/aed/mission_status`

## 오디오 출력 계층

`audio_output.AudioOutput`은 기본적으로 PC 시스템 오디오를 사용합니다.
`paplay`, `pw-play`, `aplay` 순으로 사용 가능한 플레이어를 자동 선택하므로
OS 기본 출력 장치를 블루투스 스피커로 지정하면 됩니다. 필요할 때
`audio_backend:=create3`로 기존 Create3 `AudioNoteVector` 출력도 선택할 수
있습니다.

개발 PC에 `irobot_create_msgs`가 없으면 노드가 종료되지 않고
`cmd_audio_fallback`에 `BEEP`/`STOP` 문자열을 발행합니다. 이 대체 출력은
시험용이며 실제 소리는 나지 않습니다.

## 단발 스피커 시험

```bash
ros2 run emergency_alert siren --ros-args \
  -r __ns:=/robot1 -p audio_topic:=cmd_audio
```

단발 노드는 설정된 음계가 시스템 스피커에서 재생될 시간을 확보한 뒤 종료합니다.

## 파일 구조

```text
emergency_alert/
├── alert_logic.py              # ROS 독립 상태 전이와 음 패턴 검증
├── audio_output.py             # Create3 오디오 출력 경계
├── mission_status_alert.py     # 권장 상태 구독형 경보 노드
├── alert_mission_executor.py   # Undock+Nav2+경보 결합형 실행기
├── siren_node.py               # 단발 스피커 시험
└── robot_ids.py                # 로봇 ID·namespace 검증
```
