#!/usr/bin/env bash
set -u

# 이 저장소에서 시작한 로컬 ROS/HMI 프로세스를 빠짐없이 종료한다.
# launch 부모가 먼저 죽어 PPID가 바뀐 고아 프로세스도 실행 경로로 찾는다.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)" || exit 1
if [[ -n "${AED_WS:-}" ]]; then
  AED_WORKSPACE="$(cd "${AED_WS}" && pwd -P)" || exit 1
elif [[ -f "${SCRIPT_DIR}/../tools/aliases.sh" ]]; then
  # 저장소의 tools/stop_aed_processes.sh를 직접 실행한 경우.
  AED_WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd -P)" || exit 1
else
  # install/aed_bringup/lib/aed_bringup에 설치된 ROS 실행 파일인 경우.
  AED_WORKSPACE="$(cd "${SCRIPT_DIR}/../../../.." && pwd -P)" || exit 1
fi
AED_INSTALL_PREFIX="${AED_WORKSPACE}/install/"
AED_FRONTEND_PREFIX="${AED_WORKSPACE}/src/aed_hmi/frontend/"
ROS2_CLI_PREFIX="/usr/bin/python3 /opt/ros/humble/bin/ros2 "
ROS_LIB_MARKER="/opt/ros/humble/lib/"
ROS_DAEMON_MARKER="ros2cli.daemon.daemonize"
CURRENT_UID="$(id -u)"

if [[ ! -f "${AED_WORKSPACE}/tools/aliases.sh" \
   || ! -d "${AED_WORKSPACE}/src/aed_bringup" ]]; then
  echo "AED 워크스페이스 검증 실패: ${AED_WORKSPACE}" >&2
  exit 1
fi

is_aed_process() {
  local process_args=$1
  case "${process_args}" in
    "/usr/bin/python3 ${AED_INSTALL_PREFIX}"*) return 0 ;;
    "${AED_WORKSPACE}/install/"*) return 0 ;;
    "${ROS2_CLI_PREFIX}"*) return 0 ;;
    *"${ROS_LIB_MARKER}"*) return 0 ;;
    *"${ROS_DAEMON_MARKER}"*) return 0 ;;
    *"${AED_FRONTEND_PREFIX}"*) return 0 ;;
    *) return 1 ;;
  esac
}

collect_targets() {
  local candidate_pid candidate_args
  while read -r candidate_pid candidate_args; do
    [[ -n "${candidate_pid:-}" ]] || continue
    [[ "${candidate_pid}" != "$$" && "${candidate_pid}" != "${PPID}" ]] \
      || continue
    if is_aed_process "${candidate_args}"; then
      printf '%s\n' "${candidate_pid}"
    fi
  done < <(ps -u "${CURRENT_UID}" -o pid=,args=)
}

verify_target() {
  local target_pid=$1 current_args
  current_args="$(ps -p "${target_pid}" -o args= 2>/dev/null)" || return 1
  [[ -n "${current_args}" ]] || return 1
  if ! is_aed_process "${current_args}"; then
    echo "PID ${target_pid} 검증 실패: ${current_args}" >&2
    return 2
  fi
  printf '  PID %s %s\n' "${target_pid}" "${current_args}"
}

stop_ros_daemon() {
  if command -v ros2 >/dev/null 2>&1; then
    timeout 5 ros2 daemon stop >/dev/null 2>&1 || true
  fi
}

stop_ros_daemon

mapfile -t target_pids < <(collect_targets | sort -n -u)
if (( ${#target_pids[@]} == 0 )); then
  echo "종료할 AED/ROS 프로세스가 없습니다."
  exit 0
fi
if (( ${#target_pids[@]} > 100 )); then
  echo "종료 대상이 비정상적으로 많아 중단합니다: ${#target_pids[@]}개" >&2
  exit 1
fi

echo "AED/ROS 프로세스 ${#target_pids[@]}개 종료 중..."
verified_pids=()
for target_pid in "${target_pids[@]}"; do
  if verify_target "${target_pid}"; then
    verified_pids+=("${target_pid}")
  fi
done

if (( ${#verified_pids[@]} == 0 )); then
  echo "검증 뒤 남은 종료 대상이 없습니다."
  exit 0
fi

kill -TERM "${verified_pids[@]}" 2>/dev/null || true

for _attempt in 1 2 3 4 5; do
  remaining_pids=()
  for target_pid in "${verified_pids[@]}"; do
    if kill -0 "${target_pid}" 2>/dev/null; then
      remaining_pids+=("${target_pid}")
    fi
  done
  (( ${#remaining_pids[@]} == 0 )) && break
  sleep 1
done

if (( ${#remaining_pids[@]} > 0 )); then
  echo "정상 종료되지 않은 프로세스 강제 종료: ${remaining_pids[*]}"
  for target_pid in "${remaining_pids[@]}"; do
    if verify_target "${target_pid}" >/dev/null; then
      kill -KILL "${target_pid}" 2>/dev/null || true
    fi
  done
  sleep 1
fi

final_pids=()
for target_pid in "${verified_pids[@]}"; do
  if kill -0 "${target_pid}" 2>/dev/null; then
    final_pids+=("${target_pid}")
  fi
done
if (( ${#final_pids[@]} > 0 )); then
  echo "종료되지 않은 프로세스가 있습니다: ${final_pids[*]}" >&2
  exit 1
fi

# TERM 처리 중 늦게 생성되거나 부모가 죽은 뒤 고아가 된 손자 프로세스까지
# 한 번 더 수집한다. 새 PID는 실행 경로를 재검증한 뒤 KILL한다.
mapfile -t late_pids < <(collect_targets | sort -n -u)
if (( ${#late_pids[@]} > 0 )); then
  echo "종료 중 뒤늦게 남은 프로세스 강제 종료: ${late_pids[*]}"
  for target_pid in "${late_pids[@]}"; do
    if verify_target "${target_pid}" >/dev/null; then
      kill -KILL "${target_pid}" 2>/dev/null || true
    fi
  done
  sleep 1
fi

mapfile -t leftover_pids < <(collect_targets | sort -n -u)
if (( ${#leftover_pids[@]} > 0 )); then
  echo "최종 종료되지 않은 AED/ROS 프로세스: ${leftover_pids[*]}" >&2
  exit 1
fi

echo "AED/ROS 프로세스 종료 완료."
