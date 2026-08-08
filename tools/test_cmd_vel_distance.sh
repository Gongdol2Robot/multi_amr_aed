#!/usr/bin/env bash
set -eo pipefail

test_robot_number="${1:-1}"
if [[ ! "$test_robot_number" =~ ^[12]$ ]]; then
  echo "사용법: $0 [1|2]" >&2
  exit 2
fi

test_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /etc/turtlebot4_discovery/setup.bash
export ROS_SUPER_CLIENT=True
source "$test_workspace/install/setup.bash"
set -u

mkdir -p "$test_workspace/logs"
test_log="$test_workspace/logs/cmd_vel_distance_robot${test_robot_number}_$(date +%Y%m%d_%H%M%S).log"
test_node_pid=""

stop_test_node() {
  if [[ -n "$test_node_pid" ]] && kill -0 "$test_node_pid" 2>/dev/null; then
    kill -INT "$test_node_pid" 2>/dev/null || true
    wait "$test_node_pid" 2>/dev/null || true
  fi
}

trap stop_test_node EXIT
trap 'stop_test_node; exit 130' INT TERM

echo "거리 시험 노드를 먼저 실행해 Nav2 이동 중 AMCL 위치를 저장합니다."
ros2 run sensor_recovery cmd_vel_distance_test --ros-args \
  -r "__ns:=/robot${test_robot_number}" \
  --params-file "$test_workspace/src/sensor_recovery/config/cmd_vel_distance_test.yaml" \
  -p auto_start:=false \
  > >(tee -a "$test_log") 2>&1 &
test_node_pid=$!

test_service="/robot${test_robot_number}/cmd_vel_distance_test/start"
test_service_ready=false
for _ in {1..100}; do
  if ros2 service type "$test_service" 2>/dev/null | grep -q 'std_srvs/srv/Trigger'; then
    test_service_ready=true
    break
  fi
  sleep 0.1
done
if [[ "$test_service_ready" != true ]]; then
  echo "거리 시험 start 서비스를 찾지 못했습니다." >&2
  exit 1
fi

{
  echo "1/2: Nav2로 시험 시작점 (0.80, 0.20, 90deg) 이동"
  ros2 action send_goal \
    "/robot${test_robot_number}/navigate_to_pose" \
    nav2_msgs/action/NavigateToPose \
    '{pose: {header: {frame_id: map}, pose: {position: {x: 0.80, y: 0.20, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071068, w: 0.7071068}}}}'

  read -r -p "전방 0.5m가 비어 있으면 Enter를 눌러 시험을 시작하세요. "
  echo "2/2: 0.05m/s로 10초 직진 후 오차 계산"
  test_start_response="$(
    ros2 service call "$test_service" std_srvs/srv/Trigger '{}' 2>&1
  )"
  echo "$test_start_response"
  if ! grep -q 'success=True' <<<"$test_start_response"; then
    echo "거리 시험 시작이 거부됐습니다." >&2
    exit 1
  fi
} 2>&1 | tee -a "$test_log"

wait "$test_node_pid"
test_node_pid=""
echo "로그 저장: $test_log"
