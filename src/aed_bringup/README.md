# aed_bringup

멀티 TurtleBot4 시스템의 공통 Nav2 설정과 전체 기동 파일을 관리합니다.

## Nav2 configuration

`config/nav2_aed.yaml`은 기존 미니 프로젝트에서 실기 검증한 설정을
이관한 것입니다.

- 로봇 반경: `0.20 m`
- 최대 직진 속도: `0.20 m/s`
- 최대 탐색 회전 속도: `0.7 rad/s`
- 로컬·글로벌 inflation 반경: `0.22 m`
- DWB 장애물 critic 가중치: `1.50`
- 실제 TurtleBot4 운용을 위해 `use_sim_time: false`

두 로봇은 동일한 파일을 사용하고 namespace만 `/robot1`, `/robot2`로
분리합니다.

