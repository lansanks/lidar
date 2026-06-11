#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$ROOT_DIR/script"
RUN_DIR="$SCRIPT_DIR/.run"
LOG_DIR="$SCRIPT_DIR/logs"

WAIT_SEC="${FASTLIO_WAIT_SEC:-20}"
RVIZ="${RVIZ:-false}"
CONFIG_FILE="${FASTLIO_CONFIG:-mid360.yaml}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
LIVOX_SETUP="$ROOT_DIR/radar/livox/livox_ws/install/setup.bash"
FASTLIO_SETUP="$ROOT_DIR/lio/fast_lio_ws/install/setup.bash"
PID_FILE="$RUN_DIR/fastlio.pid"
LOG_FILE="$LOG_DIR/fastlio.log"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"
ROS_HOME="${ROS_HOME:-$RUN_DIR/ros_home}"

mkdir -p "$RUN_DIR" "$LOG_DIR" "$ROS_LOG_DIR" "$ROS_HOME"
export ROS_LOG_DIR ROS_HOME

fail() {
  echo "FAIL fastlio: $*" >&2
  exit 1
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "missing command: $1"
}

source_ros() {
  [[ -f "$ROS_SETUP" ]] || fail "missing $ROS_SETUP"
  [[ -f "$LIVOX_SETUP" ]] || fail "missing $LIVOX_SETUP"
  [[ -f "$FASTLIO_SETUP" ]] || fail "missing $FASTLIO_SETUP"

  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  # shellcheck disable=SC1090
  source "$LIVOX_SETUP"
  # shellcheck disable=SC1090
  source "$FASTLIO_SETUP"
  set -u

  need_cmd ros2
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

wait_for_fastlio() {
  local nodes topics
  for ((i = 0; i < WAIT_SEC; i++)); do
    pid_running || fail "stopped, see $LOG_FILE"
    nodes="$(timeout 2 ros2 node list 2>/dev/null || true)"
    topics="$(timeout 2 ros2 topic list 2>/dev/null || true)"
    if grep -qx "/laser_mapping" <<<"$nodes" && grep -qx "/cloud_registered" <<<"$topics"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_fastlio() {
  if pid_running; then
    return 0
  fi

  : >"$LOG_FILE"
  nohup ros2 launch fast_lio mapping.launch.py "config_file:=$CONFIG_FILE" "rviz:=$RVIZ" >"$LOG_FILE" 2>&1 &
  echo "$!" >"$PID_FILE"
}

source_ros
start_fastlio
wait_for_fastlio || fail "ROS node/topic not ready, see $LOG_FILE"

echo "OK fastlio"
