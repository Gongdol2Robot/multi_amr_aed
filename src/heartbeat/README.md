# heartbeat

Mission Manager와 `/robot1`, `/robot2` 사이의 양방향 Heartbeat를 담당합니다.

- 기본 발행 주기: 1초
- 3초 미수신: 통신 이상 의심
- 5초 미수신: `NETWORK_LOST` 확정

실제 임계값은 네트워크 실험 후 확정합니다.
