# mission_manager

담당: 김재엽

확정된 응급 이벤트마다 한 대의 TurtleBot4만 AED 전달 로봇으로 배정합니다.

## Selection and recovery policy

1. 통신·Localization·Nav2·비상정지 상태가 정상인 로봇만 후보로 사용
2. 유효한 Nav2 경로를 가진 후보를 예상 경로비용 순으로 정렬
3. 최우선 로봇 한 대만 출동시키고 다른 로봇은 대기
4. `BLOCKED`, `NETWORK_LOST`, `NAVIGATION_ERROR` 수신 시 수행 로봇 제외
5. 동일 event에 assignment version을 증가시켜 대체 로봇에 재할당
6. 후보가 없으면 `RECOVERY_WAIT`로 유지하고 로봇 상태 복구를 계속 감시
7. 가용 로봇이 생기면 version을 증가시킨 새 임무로 자동 복구

복구된 로봇의 이전 상태 메시지는 assignment version이 다르므로 동일 임무를
자동 재개할 수 없습니다.

응급 이벤트는 AED 도착 전까지 종료되지 않습니다. 두 로봇이 동시에 불가한
상태는 일시적인 복구 대기 상태로 취급합니다.
