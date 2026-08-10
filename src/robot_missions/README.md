# robot_missions

배정된 AED 전달 임무를 Nav2로 수행하고 typed `MissionStatus`를 발행합니다.

- 새로운 assignment version을 수신하면 기존 Goal 취소
- 출동·주행·도착·취소·Nav2 실패 상태 보고
- 실패 후 자체적으로 이전 Goal을 재개하지 않음

```bash
ros2 run robot_missions mission_executor --ros-args \
  -r __ns:=/robot1 -p robot_id:=robot1
```

Polygon 왕복 수색 중 비전 검출이 들어오면 수색을 취소하고 대상 앞으로
접근하는 노드는 다음처럼 실행합니다.

```bash
ros2 run robot_missions search_and_detect --ros-args \
  -r __ns:=/robot1
```
