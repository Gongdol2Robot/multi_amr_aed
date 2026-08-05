# aed_bringup

통합 담당: 이현민

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

## Central dispatch

각 로봇 PC에서 Nav2를 따로 실행한 뒤 중앙 PC에서 경로비용 계산과 미션 배정을
실행합니다. 이 런치는 Nav2나 RViz를 실행하지 않습니다.

```bash
ros2 launch aed_bringup central_dispatch.launch.py
```
