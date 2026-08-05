# Module ownership

기준 문서: [기획안 최종 v3.0](https://app.notion.com/p/3b35af693d3581f8a95feaa09b63a4d7)

| 담당자 | 담당 영역 | 주 패키지 | 핵심 산출물 |
|---|---|---|---|
| 김지훈 | 호모그래피·위치 검증·SLAM 보조 | `emergency_location_mapper`, `aed_vision` | 카메라-지도 매핑, 응급 목표점, 위치 오차 검증 |
| 이현민 | 목각인형·사람 Vision, 전체 통합 | `aed_vision`, `aed_interfaces`, `aed_bringup` | Vision 모델/노드, 공통 인터페이스, 통합 launch·시험 |
| 김재엽 | 거리·경로비용 비교와 우선 로봇 선정 | `mission_manager` | 경로비용 모듈, 후보 순위, 배정 결과 |
| 김영기 | 네트워크·Nav2 서버 장애 복구 | `amr_recovery` | Heartbeat, Nav2 Health Check, Recovery Manager |
| 박재현 | LiDAR 장애 감지·대처 | `sensor_recovery`, `robot_state_monitor` | LiDAR Watchdog, 센서 상태, 장애 시험 |
| 김민성 | AED 도착 후 구조 인력 호출·안내 | `helper_mission`, `emergency_alert` | 사람 호출, 도움 요청, 현장 안내 미션 |

## Integration rule

- 각 담당자는 모듈 단위시험과 입력·출력 인터페이스 검증까지 완료한다.
- 통합 담당자는 검증된 모듈을 연결하고 통합 launch와 시나리오 시험을 관리한다.
- 인터페이스 오류는 해당 모듈 담당자와 공동 수정한다.
- 개별 미완성 기능을 통합 담당자에게 넘기지 않는다.

## Scaffold entry points

| 패키지 | 실행 파일 |
|---|---|
| `emergency_location_mapper` | `location_mapper` |
| `robot_state_monitor` | `robot_state_monitor` |
| `amr_recovery` | `recovery_manager` |
| `sensor_recovery` | `sensor_health_monitor` |
| `helper_mission` | `helper_mission_controller` |
| `event_logger` | `event_logger` |
| `aed_hmi` | `hmi_node` |
