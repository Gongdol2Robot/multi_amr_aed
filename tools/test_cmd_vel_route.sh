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
test_log="$test_workspace/logs/cmd_vel_route_robot${test_robot_number}_$(date +%Y%m%d_%H%M%S).log"
test_node_pid=""
test_namespace="/robot${test_robot_number}"
test_start_service="$test_namespace/cmd_vel_route/start"
test_stop_service="$test_namespace/cmd_vel_route/stop"
test_route="$test_workspace/src/sensor_recovery/config/robot1_undock_to_goal.yaml"

stop_test_node() {
  if ros2 service type "$test_stop_service" 2>/dev/null | grep -q 'Trigger'; then
    ros2 service call "$test_stop_service" std_srvs/srv/Trigger '{}' \
      >/dev/null 2>&1 || true
  fi
  if [[ -n "$test_node_pid" ]] && kill -0 "$test_node_pid" 2>/dev/null; then
    kill -INT "$test_node_pid" 2>/dev/null || true
    wait "$test_node_pid" 2>/dev/null || true
  fi
}

trap stop_test_node EXIT
trap 'stop_test_node; exit 130' INT TERM

if [[ "$test_robot_number" != "1" ]]; then
  echo "현재 실측 route는 robot1 전용입니다." >&2
  exit 1
fi

echo "1/2: 저장 경로 follower 실행 (dock/undock은 실행하지 않음)"
ros2 run sensor_recovery cmd_vel_route_follower --ros-args \
  -r "__ns:=$test_namespace" \
  --params-file "$test_workspace/src/sensor_recovery/config/cmd_vel_route.yaml" \
  -p "route_file:=$test_route" \
  > >(tee -a "$test_log") 2>&1 &
test_node_pid=$!

test_service_ready=false
for _ in {1..150}; do
  if ros2 service type "$test_start_service" 2>/dev/null | grep -q 'Trigger'; then
    test_service_ready=true
    break
  fi
  sleep 0.1
done
if [[ "$test_service_ready" != true ]]; then
  echo "route follower start 서비스를 찾지 못했습니다." >&2
  exit 1
fi

echo "AMCL·map·depth 입력 준비를 확인하는 중..."
test_inputs_ready=false
for _ in {1..150}; do
  if grep -q 'AMCL pose active:' "$test_log" \
    && grep -q 'Static-map route validation passed' "$test_log" \
    && grep -q 'Depth stream active:' "$test_log"; then
    test_inputs_ready=true
    break
  fi
  if grep -q 'Static-map route validation failed' "$test_log"; then
    echo "저장 경로의 map 안전성 검사를 통과하지 못했습니다. 로그: $test_log" >&2
    exit 1
  fi
  sleep 0.1
done
if [[ "$test_inputs_ready" != true ]]; then
  echo "15초 안에 AMCL·map·depth 입력이 모두 준비되지 않았습니다." >&2
  echo "mapnav 1이 완전히 활성화됐는지 확인하세요. 로그: $test_log" >&2
  exit 1
fi

echo "사전 입력 확인 완료: AMCL·map·depth 정상"
read -r -p "로봇을 시작 위치에 직접 배치했고 경로와 비상정지가 준비됐으면 Enter: "
echo "2/2: LiDAR를 켠 상태로 cmd_vel 저장 경로 주행 시작"
test_start_response="$(
  ros2 service call "$test_start_service" std_srvs/srv/Trigger '{}' 2>&1
)"
echo "$test_start_response" | tee -a "$test_log"
if ! grep -q 'success=True' <<<"$test_start_response"; then
  echo "경로 시험 시작이 거부됐습니다. 로그: $test_log" >&2
  exit 1
fi

while kill -0 "$test_node_pid" 2>/dev/null; do
  if grep -q 'ROUTE_RESULT state=' "$test_log"; then
    break
  fi
  sleep 0.2
done

stop_test_node
test_node_pid=""
echo "경로 시험 종료. 로그 저장: $test_log"
