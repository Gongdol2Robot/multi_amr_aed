#!/usr/bin/env bash
# explore_lite 자동 탐사 실행
#
# 사용법:
#   tools/explore.sh        # robot1 탐사
#   tools/explore.sh 2      # robot2 탐사
#
# 먼저 SLAM과 Nav2가 떠 있어야 한다. explore_lite는 지도를 직접 만들지 않고
# 미탐색 경계로 갈 목표만 Nav2에 던진다.
#
#   ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot2
#   ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot2 \
#     params_file:=src/aed_bringup/config/nav2_aed.yaml
#
# explore_lite가 제공하는 explore.launch.py는 params 인자를 받지 않고,
# 기본 params.yaml의 루트 키가 `explore_node:`라 네임스페이스가 붙으면
# 파라미터가 조용히 무시된다. 그래서 노드를 직접 실행한다.

set -euo pipefail

ROBOT_NUMBER="${1:-1}"
if [[ ! "$ROBOT_NUMBER" =~ ^([0-9]|1[01])$ ]]; then
  echo "오류: 로봇 번호는 0~11이어야 합니다." >&2
  exit 2
fi

ROBOT_NS="/robot${ROBOT_NUMBER}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${AED_WS:-$(dirname "$SCRIPT_DIR")}"
PARAMS="$WORKSPACE/src/aed_bringup/config/explore_aed.yaml"

if [[ ! -f "$PARAMS" ]]; then
  echo "오류: $PARAMS 가 없습니다." >&2
  exit 2
fi

# 탐사 목표를 받아 줄 Nav2가 없으면 explore는 조용히 아무것도 하지 않는다.
echo "Nav2 확인 중: $ROBOT_NS/navigate_to_pose"
if ! timeout 40 ros2 action info "$ROBOT_NS/navigate_to_pose" 2>/dev/null \
    | grep -qE '^Action servers: [1-9]'; then
  echo "오류: $ROBOT_NS/navigate_to_pose 액션 서버가 없습니다." >&2
  echo "Nav2를 먼저 띄우세요." >&2
  exit 1
fi

echo "탐사 시작: $ROBOT_NS"
echo "  중단하려면 Ctrl-C. 지도는 SLAM 터미널에서 savemap 으로 저장합니다."

# tf는 네임스페이스 안에 발행되므로 리매핑이 없으면 변환을 못 찾는다.
exec ros2 run explore_lite explore --ros-args \
  -r __ns:="$ROBOT_NS" \
  -r /tf:="$ROBOT_NS/tf" \
  -r /tf_static:="$ROBOT_NS/tf_static" \
  --params-file "$PARAMS"
