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

`path_event_id`는 어떤 응급 이벤트에 대한 진단값인지 구분합니다. 운영 배정은
`emergency_mission_manager`가 두 Nav2 Planner에 직접 요청한 실제 경로와 ETA로
판단하며, 이 모니터의 경로비용은 RobotState/HMI 진단용입니다. 경로 생성이
실패하면 3초 간격으로 재시도합니다.

Robot1 단독 거리 계산 시험:

```bash
ros2 run robot_state_monitor robot_state_monitor --ros-args \
  -p robot_ids:="[robot1]"
```

단독 시험에서는 `emergency_mission_manager`를 실행하지 않습니다.
`/robot1/amcl_pose`와 `/robot1/compute_path_to_pose`가 먼저 준비되어 있어야
합니다.

중앙 PC 실행:

```bash
ros2 launch aed_hmi hmi_runtime.launch.py start_backend:=false
```

중앙 노드를 먼저 켠 뒤 RViz에서 **Publish Point**로 지도 좌표를 클릭하면
`/clicked_point`, `/robot1/clicked_point`, `/robot2/clicked_point` 중 어느
토픽으로 들어와도 새 이벤트로 변환하여 두 로봇의 경로를 다시 계산합니다.
`Nav2 Goal`은 사용하지 않습니다. 기본값은 거리 비교만 수행하고 로봇을
움직이지 않습니다.

```bash
ros2 topic echo /aed/path_distance/robot1
ros2 topic echo /aed/path_distance/robot2
ros2 topic echo /emergency/selected_robot
```

경로를 계산할 수 없는 로봇의 거리에는 `nan`이 발행됩니다. 실제로 선정된
로봇에 임무를 배정하려면 다음처럼 실행합니다.

정지 중에는 AMCL pose가 반복 발행되지 않으므로 마지막 수신 위치를 계속
사용합니다. 엄격한 pose 시간 제한은 `allow_stale_pose:=false`로 켤 수
있습니다.

기본 `use_planner_start:=true`에서는 각 Nav2 Planner가 TF에서 자신의 현재
시작 위치를 직접 가져오므로 중앙 노드의 오래된 AMCL pose를 경로 시작점으로
강제하지 않습니다.

```bash
ros2 launch multi_robot_emergency central_dispatch.launch.py \
  dispatch_enabled:=true
```

시험 이벤트:

```bash
ros2 topic pub --once /aed/emergency_event aed_interfaces/msg/EmergencyEvent \
  "{event_id: test-001, location: {header: {frame_id: map}, point: {x: 1.2, y: 2.4}}, confidence: 1.0, consecutive_detections: 1, status: 1, source_id: manual, location_source: manual, location_valid: true}"
```

결과 확인:

```bash
ros2 topic echo /aed/robot_state
ros2 topic echo /robot1/mission_assignment
ros2 topic echo /robot2/mission_assignment
```
