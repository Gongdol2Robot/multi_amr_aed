# mission_manager

확정된 응급 이벤트와 각 로봇의 상태를 받아 동적으로 역할을 배정합니다.

정책:

1. `AVAILABLE`이며 pose의 frame이 있는 로봇만 후보로 사용
2. 응급 위치에 가장 가까운 로봇에 `ROLE_AED_DELIVERY` 배정
3. 두 번째 로봇에 `ROLE_GUIDE` 배정
4. 길안내 로봇은 설정된 대기 위치를 방문한 뒤 응급 위치로 이동

```bash
ros2 run mission_manager mission_manager --ros-args \
  -p robot_ids:="[robot1, robot2]" \
  -p guide_wait_x:=0.0 -p guide_wait_y:=0.0
```

