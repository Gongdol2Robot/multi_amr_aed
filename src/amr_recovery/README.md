# amr_recovery

담당: 김영기

네트워크와 Nav2 서버 장애를 진단하고 자동 복구합니다.

## Planned nodes

- `heartbeat_node`: Mission Manager와 로봇 사이 1초 주기 양방향 Heartbeat
- `network_monitor`: 3초 이상 미수신 의심, 5초 이상 `NETWORK_LOST` 확정
- `nav2_health_check`: Nav2 lifecycle·action server 상태 감시
- `recovery_manager`: 관련 노드·서비스 재시작과 재할당 요청
- `safety_watchdog`: 관제 연결 상실 시 Goal 취소와 로봇 안전 정지

통신이 복구되어도 이전 assignment version은 자동 재개하지 않습니다.
