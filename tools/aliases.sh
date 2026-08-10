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

slam() {
  local n=${1:-1}
  ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot$n
}

drive() {
  local n=${1:-1}
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r /cmd_vel:=/robot$n/cmd_vel
}

# SLAM과 Nav2가 떠 있어야 한다. 미탐색 경계로 갈 목표만 Nav2에 던진다.
explore() {
  "$AED_WS/tools/explore.sh" "${1:-1}"
}

# 지도는 slam_toolbox가 살아 있는 동안에만 저장할 수 있다. SLAM 터미널을
# 먼저 끄면 그때까지 돈 것이 전부 사라진다.
#
# slam_toolbox의 save_map 서비스는 쓰지 않는다. 내부에서 map_saver를 띄우는데
# 대기 시간이 2초로 고정이라 discovery server 환경에서는 지도를 못 받고
# result=255로 실패한다. map_saver_cli를 직접 부르고 시간을 넉넉히 준다.
savemap() {
  local n=${1:-1} name=${2:-map}
  mkdir -p "$AED_WS/maps"
  ros2 run nav2_map_server map_saver_cli \
    -f "$AED_WS/maps/$name" -t "/robot$n/map" \
    --ros-args -p save_map_timeout:=15.0 || return 1
  # posegraph는 이어서 매핑할 때만 필요하다. 수백 MB라 git에는 올리지 않는다.
  ros2 service call /robot$n/slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '$AED_WS/maps/$name'}"
}

loc() {
  local n=${1:-1} map=${2:-$AED_WS/maps/map.yaml}
  ros2 launch turtlebot4_navigation localization.launch.py \
    namespace:=/robot$n map:="$map"
}

# Polygon 왕복 수색 + 검출 접근. namespaced TF 토픽 리매핑을 항상 포함한다.
searchdetect() {
  local n=${1:-1}
  shift 2>/dev/null || true
  aedenv
  ros2 run robot_missions search_and_detect --ros-args \
    -r __ns:=/robot$n \
    -r /tf:=/robot$n/tf \
    -r /tf_static:=/robot$n/tf_static \
    "$@"
}

# 두 로봇이 지도는 공유하지만 Dock 위치가 달라 초기 위치는 각자 다르다.
# 좌표는 src/aed_bringup/config/dock_poses.yaml 에서 읽는다.
#   initpose 2            robot2 의 Dock 위치를 발행
#   initpose 1 --record   현재 위치를 robot1 항목에 기록
initpose() {
  local n=${1:-1}; shift 2>/dev/null || true
  python3 "$AED_WS/tools/initpose.py" "$n" "$@"
}

nav() {
  local n=${1:-1}
  if ! python3 "$AED_WS/tools/preflight.py" \
      --localization --namespace "robot$n"; then
    echo "Nav2 시작 중단: loc $n -> initpose $n 순서로 복구하세요."
    return 1
  fi
  ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot$n \
    params_file:="$AED_WS/src/aed_bringup/config/nav2_aed.yaml"
}

rv() {
  local n=${1:-1}
  ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot$n
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

bagrec() {
  local n=${1:-1}
  mkdir -p "$AED_WS/bags"
  ros2 bag record -o "$AED_WS/bags/$(date +%m%d_%H%M)" \
    --compression-mode file --compression-format zstd \
    /robot$n/oakd/rgb/image_raw/compressed /robot$n/scan /robot$n/odom \
    /robot$n/tf /robot$n/tf_static /robot$n/mission_assignment \
    /robot$n/mission_status /aed/emergency_event /aed/robot_state
}

alias mstate='ros2 topic echo /aed/robot_state'
alias estate='ros2 topic echo /aed/emergency_event'
