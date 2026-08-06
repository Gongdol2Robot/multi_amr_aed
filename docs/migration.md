# Legacy migration notes

## Migrated

| New component | Legacy source | Decision |
|---|---|---|
| `emergency_alert.siren_node` | `turtlebot4_beep.beep_node` | `/robot2` 하드코딩 제거, 토픽·반복·음 길이 파라미터화 |
| `robot_missions.mission_executor` | `mini_proj.mission_controller` | RC카 추종을 제거하고 취소 가능한 공통 Nav2 미션 실행기로 분리 |
| `mission_manager` | 신규 분리 | 로봇 위치 비교와 AED·길안내 역할 배정을 중앙 관리자 책임으로 분리 |
| `aed_interfaces` | 신규 분리 | 노드 사이 문자열 결합 대신 명시적 ROS 2 메시지 계약 정의 |
| `aed_bringup/config/nav2_aed.yaml` | `mini_proj/config/nav2_mini_proj.yaml` | 실기 주행 설정 이관, 실제 로봇용 `use_sim_time: false` 적용 |

## Deliberately not migrated yet

- RC카 검출 클래스와 RC카 추종 상태 머신
- `/robot2`에 고정된 액션 및 토픽 이름
- 이전 실습장의 지도와 호모그래피 측정값
- TensorRT engine, ONNX, YOLO 학습 결과와 원본 데이터셋
- `build`, `install`, `log`, 가상환경 및 Python 캐시

## Next candidates

1. Nav2 `ComputePathToPose` 결과로 실제 경로비용 계산
2. 양방향 Heartbeat와 로봇 측 Watchdog 구현
3. AMCL/배터리/Nav2 상태를 `robot_state_monitor`에서 발행
4. 장애·복구 대기·재할당 이력 SQLite 저장과 HMI 복구 지원 알림 구현
5. AED 도착 후 사람 부재 판정과 `helper_mission` 호출·안내 구현
