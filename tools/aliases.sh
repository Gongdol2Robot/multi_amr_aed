# Multi-AMR AED 프로젝트 공통 단축 명령
# 설치:
#   echo 'source ~/rokey_ws/multi_amr_aed/tools/aliases.sh' >> ~/.bashrc
#   source ~/.bashrc

export AED_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

aedenv() {
  source /etc/turtlebot4_discovery/setup.bash
  export ROS_SUPER_CLIENT=True
  source "$AED_WS/install/setup.bash"
}

roskill() {
  local uid workspace_pattern
  uid="$(id -u)"
  workspace_pattern="$AED_WS/install/.*/lib/"
  echo "ROS2/Nav2/AED 프로세스 종료 중..."
  timeout 5 ros2 daemon stop >/dev/null 2>&1 || true
  pkill -TERM -u "$uid" -f '(/usr/bin/python3 )?/opt/ros/humble/bin/ros2([[:space:]]|$)' 2>/dev/null || true
  pkill -TERM -u "$uid" -f '/opt/ros/humble/lib/' 2>/dev/null || true
  pkill -TERM -u "$uid" -f "$workspace_pattern" 2>/dev/null || true
  pkill -TERM -u "$uid" -f 'ros2cli\.daemon\.daemonize' 2>/dev/null || true
  sleep 3
  pkill -KILL -u "$uid" -f '/opt/ros/humble/lib/' 2>/dev/null || true
  pkill -KILL -u "$uid" -f "$workspace_pattern" 2>/dev/null || true
  pkill -KILL -u "$uid" -f 'ros2cli\.daemon\.daemonize' 2>/dev/null || true
  echo "종료 완료."
}

robotstart() {
  export ROBOT_NUMBER="${1:-1}"
  export WEBCAM_DEVICE="${2:-0}"
  terminator --no-dbus --config "$AED_WS/tools/terminator_robot.conf" \
    --layout robot_session >/dev/null 2>&1 &
  disown
}

pf() {
  local n=${1:-1} host=${2:-192.168.0.2}
  python3 "$AED_WS/tools/preflight.py" --namespace "robot$n" --host "$host"
}

fit() {
  local n=${1:-1}
  ROBOT_NS="/robot$n" python3 "$AED_WS/tools/check_fit.py" --ros-args \
    -r /tf:=/robot$n/tf -r /tf_static:=/robot$n/tf_static
}

undock() {
  local n=${1:-1}
  ros2 action send_goal /robot$n/undock irobot_create_msgs/action/Undock "{}"
}

dock() {
  local n=${1:-1}
  ros2 action send_goal /robot$n/dock irobot_create_msgs/action/Dock "{}"
}

loc() {
  local n=${1:-1} map=${2:-$AED_WS/maps/map.yaml}
  ros2 launch turtlebot4_navigation localization.launch.py \
    namespace:=/robot$n map:="$map"
}

initpose() {
  local n=${1:-1}
  ros2 topic pub --times 3 /robot$n/initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
}

nav() {
  local n=${1:-1}
  if ! python3 "$AED_WS/tools/preflight.py" \
      --localization --namespace "robot$n"; then
    echo "Nav2 시작 중단: loc $n -> initpose $n 순서로 복구하세요."
    return 1
  fi
  ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot$n
}

rv() {
  local n=${1:-1}
  ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot$n
}

vision() {
  local dev=${1:-0}
  ros2 run aed_vision webcam_publisher --ros-args -p device:="$dev"
}

manager() {
  ros2 run mission_manager mission_manager
}

executor() {
  local n=${1:-1}
  ros2 run robot_missions mission_executor --ros-args \
    -r __ns:=/robot$n -p robot_id:="robot$n"
}

siren() {
  local n=${1:-1}
  ros2 run emergency_alert siren --ros-args \
    -r __ns:=/robot$n -p audio_topic:=cmd_audio
}

recover() {
  "$AED_WS/tools/nav_recovery.sh" "${1:-1}"
}

survey() {
  local label=${1:?"사용: survey <라벨> [로봇번호]"} n=${2:-1}
  ROBOT_NS="/robot$n" python3 "$AED_WS/tools/survey_point.py" "$label" \
    --ros-args -r /tf:=/robot$n/tf -r /tf_static:=/robot$n/tf_static
}

bagrec() {
  local n=${1:-1}
  mkdir -p "$AED_WS/bags"
  ros2 bag record -o "$AED_WS/bags/$(date +%m%d_%H%M)" \
    --compression-mode file --compression-format zstd \
    /robot$n/oakd/rgb/image_raw/compressed /robot$n/scan /robot$n/odom \
    /robot$n/tf /robot$n/tf_static /robot$n/mission_assignment \
    /robot$n/mission_status /aed/emergency_event /aed/robot_state
}

alias detect='vision'
alias mstate='ros2 topic echo /aed/robot_state'
alias estate='ros2 topic echo /aed/emergency_event'
