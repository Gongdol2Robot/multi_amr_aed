# Fault-recovery policy

기준 문서: [장애 복구형 시나리오](https://app.notion.com/p/3b35af693d3581f8a95feaa09b63a4d7)

## Invariants

- 하나의 `event_id`에는 활성 AED 수행 로봇이 최대 한 대만 존재한다.
- 재할당마다 `assignment_version`을 증가시킨다.
- 이전 version의 상태와 결과는 현재 임무 상태를 변경할 수 없다.
- 장애에서 복구된 로봇은 이전 Goal을 자동 재개하지 않는다.
- 후보가 없으면 `RECOVERY_WAIT`로 유지하고 가용 상태 복구를 계속 감시한다.
- 로봇이 복구되면 증가된 assignment version으로 새 Goal을 발행한다.
- 응급 이벤트는 `ARRIVED` 또는 `COMPLETED` 전까지 terminal 상태가 되지 않는다.

## Initial thresholds

- Heartbeat: 1초 주기
- 통신 이상 의심: 3초 미수신
- 네트워크 장애 확정: 5초 미수신
- 진행 장애 후보: 최근 10초간 목표거리 감소 0.1m 미만
- 경로 생성 실패: 3회 연속

임계값은 실물 시험 결과를 기록한 뒤 조정한다. 일시 장애에는 Nav2 복구를
우선 적용하고, 지속 장애가 확정된 경우에만 대체 로봇을 출동시킨다.

## No-terminal-failure principle

두 로봇이 동시에 불가하더라도 AED 임무를 실패로 종료하지 않는다. 시스템은
안전 정지 상태에서 복구를 기다리고, HMI에는 장애 원인과 복구 지원 필요성을
표시한다. 운영자 조치는 임무 포기가 아니라 로봇·경로·네트워크를 가용 상태로
되돌리기 위한 복구 절차다. 가용 로봇이 확인되면 Mission Manager가 새 version의
임무를 발행하고 AED 도착까지 복구 사이클을 지속한다.
