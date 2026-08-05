# sensor_recovery

담당: 박재현

LiDAR `/scan`의 수신 상태와 데이터 유효성을 감시하고 센서 장애에 대응합니다.

## Planned nodes

- `lidar_watchdog`: `/scan` timeout과 비정상 데이터 판정
- `sensor_health_monitor`: 센서 상태를 `RobotState`에 반영
- `lidar_recovery`: 로봇 안전 정지, 센서 노드 재시작, 대체 로봇 요청

핵심 산출물은 LiDAR Watchdog, Sensor Health Monitor와 장애 주입 시험 결과입니다.
