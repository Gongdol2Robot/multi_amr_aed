# rokey_ws 미니프로젝트 공통 단축 명령
#
# 설치 (조원 공통):
#   echo 'source ~/rokey_ws/tools/aliases.sh' >> ~/.bashrc
#   source ~/.bashrc
#
# 전제: /etc/turtlebot4_discovery/setup.bash 와 install/setup.bash 는
# 각자 .bashrc 에서 source 하고 있어야 한다 (없으면 `env` 부터).
#
# 기동 순서:  env → pf → undock → loc → initpose → fit → bagrec → detect → nav → go

env() {
  source /etc/turtlebot4_discovery/setup.bash
  export ROS_SUPER_CLIENT=True
  source ~/rokey_ws/install/setup.bash
}

# 이 노트북에서 사용자가 실행한 ROS2 작업을 모두 종료한다.
# Pi의 turtlebot4.service와 일반 터미널/개발 도구는 건드리지 않는다.
roskill() {
  local uid workspace_pattern
  uid="$(id -u)"
  workspace_pattern="$HOME/rokey_ws/install/.*/lib/"

  echo "ROS2/Nav2/RViz/디텍터 프로세스 종료 중..."
  # XML-RPC 소켓을 깨끗하게 지워 다음 ros2 명령이 죽은 daemon을 물지 않게 한다.
  timeout 5 ros2 daemon stop >/dev/null 2>&1 || true
  pkill -TERM -u "$uid" -f '(/usr/bin/python3 )?/opt/ros/humble/bin/ros2([[:space:]]|$)' 2>/dev/null || true
  pkill -TERM -u "$uid" -f '/opt/ros/humble/lib/' 2>/dev/null || true
  pkill -TERM -u "$uid" -f "$workspace_pattern" 2>/dev/null || true
  pkill -TERM -u "$uid" -f 'ros2cli\.daemon\.daemonize' 2>/dev/null || true

  # launch 종료 뒤 고아로 남는 lifecycle manager 등에 정상 종료 시간을 준다.
  sleep 3
  pkill -KILL -u "$uid" -f '/opt/ros/humble/lib/' 2>/dev/null || true
  pkill -KILL -u "$uid" -f "$workspace_pattern" 2>/dev/null || true
  pkill -KILL -u "$uid" -f 'ros2cli\.daemon\.daemonize' 2>/dev/null || true
  echo "종료 완료 (Pi 기본 서비스는 유지)."
}

# Terminator 한 창에 2열 x 3행으로 전체 운용 프로세스를 순차 기동한다.
# 사용법: robotstart [로봇 번호] [웹캠 장치 번호]
robotstart() {
  export ROBOT_NUMBER="${1:-2}"
  export WEBCAM_DEVICE="${2:-2}"
  terminator --no-dbus \
    --config "$HOME/rokey_ws/tools/terminator_robot.conf" \
    --layout robot_session >/dev/null 2>&1 &
  disown
}

alias pf='python3 ~/rokey_ws/tools/preflight.py'
alias fit='python3 ~/rokey_ws/tools/check_fit.py --ros-args -r /tf:=/robot2/tf -r /tf_static:=/robot2/tf_static'

undock() {
  local n=${1:-2}
  ros2 action send_goal /robot$n/undock irobot_create_msgs/action/Undock "{}"
}

dock() {
  local n=${1:-2}
  ros2 action send_goal /robot$n/dock irobot_create_msgs/action/Dock "{}"
}

loc() {
  local n=${1:-2} map=${2:-$HOME/rokey_ws/maps/map.yaml}
  ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot$n map:=$map
}

# undock 지점 = 맵 원점이므로 (0,0,0) 을 AMCL 초기 위치로 넣는다.
initpose() {
  local n=${1:-2}
  ros2 topic pub --times 3 /robot$n/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
}

nav() {
  local n=${1:-2}
  # map_server/AMCL과 map->base_link TF가 준비되지 않은 상태에서 Nav2를
  # 띄우면 costmap이 "map frame does not exist"로 멈춘다. 실행 전에 차단한다.
  if ! python3 "$HOME/rokey_ws/tools/preflight.py" \
      --localization --namespace "robot$n"; then
    echo "Nav2 시작 중단: loc $n -> initpose $n 순서로 복구한 뒤 다시 실행하세요."
    return 1
  fi
  # 기본 설정으로 띄우면 장애물 회피 가중치가 사실상 0 이라 벽에 박는다.
  # 이 방에 맞춘 mini_proj 설정을 기본으로 쓴다.
  ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot$n \
    params_file:=$HOME/rokey_ws/install/mini_proj/share/mini_proj/config/nav2_mini_proj.yaml
}

detect() {
  # 웹캠 장치 번호는 USB 재연결로 바뀐다 (video2 -> video3 사례). 인자로 준다.
  #   detect 3        : RGB 만 (와이파이 절약)
  #   detect 3 depth  : 압축 depth 포함 (9.5 Hz, 보정식 적용, 화면에 depth 표시)
  local dev=${1:-3}
  local extra=""
  [ "${2:-}" = "depth" ] && extra="turtlebot_depth_topic:=/robot2/oakd/stereo/image_raw/compressedDepth"
  ros2 launch mini_proj dual_view_detection.launch.py \
    webcam_device:=$dev $extra \
    turtlebot_weights:=$HOME/rokey_ws/src/wc_tb_collect_test/resource/models/yolo11n_all_data_joint_20260801_best.pt \
    webcam_weights:=$HOME/rokey_ws/src/wc_tb_collect_test/resource/models/yolo11n_combined_webcam_add_finetune_20260801_best.pt
}

bagrec() {
  ros2 bag record -o ~/rokey_ws/bags/$(date +%m%d_%H%M) \
    -d 120 --compression-mode file --compression-format zstd \
    /robot2/oakd/rgb/image_raw/compressed /camera/rgb/image_raw/compressed \
    /robot2/scan /robot2/odom /robot2/tf /robot2/tf_static \
    /mini_proj/turtlebot/car /mini_proj/webcam/car /mini_proj/webcam/car_map \
    /mini_proj/mission_state /robot2/cmd_vel /robot2/hazard_detection
}

alias go='ros2 param set /mission_controller enable_navigation true'
alias stop='ros2 param set /mission_controller enable_navigation false'
alias mstate='ros2 topic echo /mini_proj/mission_state'
