# robot_missions

배정된 AED 전달 임무를 Nav2로 수행하고 typed `MissionStatus`를 발행합니다.

- 새로운 assignment version을 수신하면 기존 Goal 취소
- 출동·주행·도착·취소·Nav2 실패 상태 보고
- 실패 후 자체적으로 이전 Goal을 재개하지 않음

```bash
ros2 run robot_missions mission_executor --ros-args \
  -r __ns:=/robot1 -p robot_id:=robot1
```
