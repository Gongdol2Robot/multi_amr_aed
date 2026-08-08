#!/usr/bin/env bash
set -eo pipefail

test_robot_number="${1:-1}"
if [[ ! "$test_robot_number" =~ ^[12]$ ]]; then
  echo "사용법: $0 [1|2]" >&2
  exit 2
fi

test_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test_log_dir="$test_workspace/logs"
test_log_file="$test_log_dir/depth_distance_robot${test_robot_number}_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$test_log_dir"
source /etc/turtlebot4_discovery/setup.bash
export ROS_SUPER_CLIENT=True
source "$test_workspace/install/setup.bash"
set -u

echo "Depth 거리 화면 실행: cmd_vel은 발행하지 않으며 로봇은 움직이지 않습니다."
echo "종료: 화면에서 q/Esc 또는 이 터미널에서 Ctrl+C"
echo "로그: $test_log_file"
ros2 run sensor_recovery depth_distance_viewer --ros-args \
  -r "__ns:=/robot${test_robot_number}" 2>&1 | tee -a "$test_log_file"
