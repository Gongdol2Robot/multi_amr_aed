# safety_watchdog

로봇 측에서 Mission Manager Heartbeat를 감시합니다. 장애 확정 시 현재 Nav2
Goal을 취소하고 정지하며, 통신 복구 후에도 이전 assignment version을 자동
재개하지 않습니다.
