#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$ROOT_DIR/script"
RUN_DIR="$SCRIPT_DIR/.run"
LOG_DIR="$SCRIPT_DIR/logs"

IFACE="${IFACE:-${1:-enp2s0}}"
HOST_CIDR="${HOST_CIDR:-${2:-192.168.1.5/24}}"
WAIT_SEC="${MID360_WAIT_SEC:-20}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
LIVOX_SETUP="$ROOT_DIR/radar/livox/livox_ws/install/setup.bash"
PID_FILE="$RUN_DIR/mid360.pid"
LOG_FILE="$LOG_DIR/mid360.log"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"
ROS_HOME="${ROS_HOME:-$RUN_DIR/ros_home}"

mkdir -p "$RUN_DIR" "$LOG_DIR" "$ROS_LOG_DIR" "$ROS_HOME"
export ROS_LOG_DIR ROS_HOME

fail() {
  echo "FAIL mid360: $*" >&2
  exit 1
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "missing command: $1"
}

source_ros() {
  [[ -f "$ROS_SETUP" ]] || fail "missing $ROS_SETUP"
  [[ -f "$LIVOX_SETUP" ]] || fail "missing $LIVOX_SETUP"

  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  # shellcheck disable=SC1090
  source "$LIVOX_SETUP"
  set -u

  need_cmd ros2
}

sudo_cmd() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

check_net() {
  local link_line
  link_line="$(ip link show dev "$IFACE" 2>/dev/null || true)"
  [[ "$link_line" =~ \<[^[:space:]]*UP ]] || return 1
  ip -o -4 addr show dev "$IFACE" | awk '{print $4}' | grep -qx "$HOST_CIDR"
}

setup_net() {
  need_cmd ip
  ip link show "$IFACE" >/dev/null 2>&1 || fail "no interface $IFACE"
  check_net && return 0

  sudo_cmd ip addr flush dev "$IFACE"
  sudo_cmd ip addr add "$HOST_CIDR" dev "$IFACE"
  sudo_cmd ip link set "$IFACE" up

  check_net || fail "net check failed ($IFACE $HOST_CIDR)"
}

pid_running() {
  [[ -f "$PID_FILE" ]] || return 1

  local pid
  pid="$(cat "$PID_FILE")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" >/dev/null 2>&1; then
    return 0
  fi

  rm -f "$PID_FILE"
  return 1
}

wait_for_node() {
  local nodes
  for ((i = 0; i < WAIT_SEC; i++)); do
    pid_running || fail "stopped, see $LOG_FILE"
    nodes="$(timeout 2 ros2 node list 2>/dev/null || true)"
    if grep -Eq '^/(livox_lidar_publisher|livox_driver_node)$' <<<"$nodes"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_mid360() {
  if pid_running; then
    return 0
  fi

  : >"$LOG_FILE"
  nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py >"$LOG_FILE" 2>&1 &
  echo "$!" >"$PID_FILE"
}

setup_net
source_ros
start_mid360
wait_for_node || fail "ROS node not ready, see $LOG_FILE"

echo "OK mid360"
