# robot_missions

Mission Manager가 발행한 `MissionAssignment`를 Nav2 목표로 실행합니다.

각 로봇 namespace 안에서 같은 노드를 실행하면 상대 이름인
`navigate_to_pose`, `mission_assignment`, `mission_status`가 자동 분리됩니다.

```bash
ros2 run robot_missions mission_executor --ros-args \
  -r __ns:=/robot1 -p robot_id:=robot1
```

길안내 역할은 `secondary_target`이 설정된 경우 첫 목적지 도착 후 두 번째
목적지를 수행합니다. 새로운 배정이 오면 진행 중인 Nav2 목표를 취소합니다.

