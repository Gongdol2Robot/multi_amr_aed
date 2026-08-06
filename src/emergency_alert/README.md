# emergency_alert

TurtleBot4 스피커로 긴급 알림음을 재생하고, AED 출동 중 알람을 제어합니다.

## AED 출동 알람

`alert_mission_executor`는 로봇별 `MissionAssignment`를 받아 다음 순서로
AED 전달 임무를 수행합니다.

1. `Undock` Action 수행
2. Undock 성공 직후 알람음 반복 시작
3. 응급 위치로 `NavigateToPose` Goal 전송
4. 도착, 주행 실패 또는 Goal 취소 시 알람 중지
5. 도착 시 `ARRIVED`, 실패 시 `NAVIGATION_ERROR`를 `/aed/mission_status`에 발행

Mission Manager는 수행 로봇의 `NAVIGATION_ERROR`를 받으면 해당 로봇을
제외하고 다음 가용 로봇에 새 assignment version을 발행합니다. 새로 배정된
로봇도 동일하게 Undock한 뒤 알람을 울리며 출동합니다.

## 여러 로봇 실행

실행기는 특정 로봇 번호나 전체 로봇 수를 가정하지 않습니다. 공통 launch에
쉼표로 구분한 ID를 전달하면 각 namespace에 실행기가 하나씩 생성됩니다.

```bash
# 2대
ros2 launch emergency_alert multi_robot_alert.launch.py \
  robot_ids:=robot1,robot2

# 4대
ros2 launch emergency_alert multi_robot_alert.launch.py \
  robot_ids:=robot1,robot2,robot3,robot4
```

로봇 ID는 중복될 수 없으며 ROS namespace로 사용할 수 있는 영문자 시작의
영문·숫자·밑줄 조합이어야 합니다. 잘못된 목록은 일부 로봇만 조용히 누락시키지
않고 launch 단계에서 오류로 종료됩니다.

> 이 패키지는 임의 개수의 로봇 실행을 지원하지만, 실제 후보 선정과 재할당
> 가능 대수는 Mission Manager의 `robot_ids` 정책에도 동일하게 반영되어야 합니다.

필요하면 로봇별 namespace에서 하나씩 직접 실행할 수도 있습니다.

```bash
ros2 run emergency_alert alert_mission_executor --ros-args \
  -r __ns:=/robot1 -p robot_id:=robot1

ros2 run emergency_alert alert_mission_executor --ros-args \
  -r __ns:=/robot2 -p robot_id:=robot2
```

> `alert_mission_executor`가 Undock과 Nav2 Goal까지 담당하므로 같은 로봇에서
> `robot_missions mission_executor`를 동시에 실행하면 안 됩니다. 동시에 실행하면
> 동일 assignment에 Nav2 Goal이 중복 전송됩니다.

기본 상대 이름은 namespace에 따라 다음처럼 해석됩니다.

- `mission_assignment` → `/robotN/mission_assignment`
- `undock` → `/robotN/undock`
- `navigate_to_pose` → `/robotN/navigate_to_pose`
- `cmd_audio` → `/robotN/cmd_audio`
- Mission status는 공통 `/aed/mission_status`

알람 속도와 음은 파라미터로 조정할 수 있습니다.

```bash
ros2 run emergency_alert alert_mission_executor --ros-args \
  -r __ns:=/robot1 -p robot_id:=robot1 \
  -p alarm_period:=0.8 -p note_duration:=0.25 \
  -p high_frequency:=1000 -p low_frequency:=440
```

## 단발 Siren

기존 `siren` 노드는 한 번의 알람 시퀀스를 재생하고 종료합니다. 토픽이
파라미터이므로 로봇별 namespace에 맞게 실행할 수 있습니다.

```bash
ros2 run emergency_alert siren --ros-args \
  -p audio_topic:=/robot1/cmd_audio
```
