# Legacy migration notes

## Migrated

| New component | Legacy source | Decision |
|---|---|---|
| `aed_vision.webcam_publisher` | `mini_proj.webcam_publisher` | AED 공통 토픽과 장치 기본값으로 변경 |
| `aed_vision.homography` | `mini_proj.homography` | 패키지 경로 변경, 현장 보정값 제거 |
| `emergency_alert.siren_node` | `turtlebot4_beep.beep_node` | `/robot2` 하드코딩 제거, 토픽·반복·음 길이 파라미터화 |
| `robot_missions.mission_executor` | `mini_proj.mission_controller` | RC카 추종을 제거하고 취소 가능한 공통 Nav2 미션 실행기로 분리 |
| `mission_manager` | 신규 분리 | 로봇 위치 비교와 AED·길안내 역할 배정을 중앙 관리자 책임으로 분리 |
| `aed_interfaces` | 신규 분리 | 노드 사이 문자열 결합 대신 명시적 ROS 2 메시지 계약 정의 |

## Deliberately not migrated yet

- RC카 검출 클래스와 RC카 추종 상태 머신
- `/robot2`에 고정된 액션 및 토픽 이름
- 이전 실습장의 지도와 호모그래피 측정값
- TensorRT engine, ONNX, YOLO 학습 결과와 원본 데이터셋
- `build`, `install`, `log`, 가상환경 및 Python 캐시

## Next candidates

1. YOLO 추론부를 `EmergencyDetection` 인터페이스로 분리
2. AMCL/배터리/통신 상태를 `robot_state_monitor`에서 발행
3. 두 로봇과 중앙 노드를 함께 시작하는 `aed_bringup` launch 작성
4. HMI와 이벤트 영속 저장 구현
