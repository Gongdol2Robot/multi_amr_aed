# robot_state_monitor

각 AMR의 위치, 배터리, 통신, Nav2 및 오류 상태를 수집·발행합니다.

LiDAR 상태 입력은 박재현의 `sensor_recovery`, 네트워크·Nav2 상태 입력은
김영기의 `amr_recovery`와 연동합니다.

현재 중앙 모니터는 `/robot1`, `/robot2`의 AMCL pose와 배터리를 수집합니다.
`/aed/emergency_event`가 `CONFIRMED`되면 양쪽
`compute_path_to_pose`에 같은 목표를 요청하고 반환된 `nav_msgs/Path`의 모든
구간 길이를 합산합니다.

계산 결과는 `/aed/robot_state`의 다음 필드로 발행합니다.

- `path_valid`
- `estimated_path_cost`
- `path_event_id`

`path_event_id`가 현재 응급 이벤트와 일치해야 Mission Manager가 해당 비용을
후보 선정에 사용합니다. 경로 생성이 실패하면 3초 간격으로 재시도합니다.

Robot1 단독 거리 계산 시험:

```bash
ros2 run robot_state_monitor robot_state_monitor --ros-args \
  -p robot_ids:="[robot1]"
```

단독 시험에서는 Mission Manager를 실행하지 않습니다. `/robot1/amcl_pose`와
`/robot1/compute_path_to_pose`가 먼저 준비되어 있어야 합니다.

중앙 PC 실행:

```bash
ros2 launch aed_bringup central_dispatch.launch.py
```

시험 이벤트:

```bash
ros2 topic pub --once /aed/emergency_event aed_interfaces/msg/EmergencyEvent \
  "{event_id: test-001, location: {header: {frame_id: map}, point: {x: 1.2, y: 2.4}}, confidence: 1.0, consecutive_detections: 1, status: 1, source_id: manual}"
```

결과 확인:

```bash
ros2 topic echo /aed/robot_state
ros2 topic echo /robot1/mission_assignment
ros2 topic echo /robot2/mission_assignment
```
