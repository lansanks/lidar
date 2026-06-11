#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$SCRIPT_DIR/.run"

stop_one() {
  local name="$1"
  local pid_file="$RUN_DIR/$name.pid"

  if [[ ! -f "$pid_file" ]]; then
    echo "OK $name: not running"
    return 0
  fi

  local pid
  pid="$(cat "$pid_file")"
  if [[ ! "$pid" =~ ^[0-9]+$ ]]; then
    rm -f "$pid_file"
    echo "OK $name: removed invalid pid file"
    return 0
  fi

  if ! kill -0 "$pid" >/dev/null 2>&1; then
    rm -f "$pid_file"
    echo "OK $name: removed stale pid file"
    return 0
  fi

  kill -INT "$pid" >/dev/null 2>&1 || true
  for _ in {1..10}; do
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      rm -f "$pid_file"
      echo "OK $name: stopped"
      return 0
    fi
    sleep 0.5
  done

  kill "$pid" >/dev/null 2>&1 || true
  for _ in {1..6}; do
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      rm -f "$pid_file"
      echo "OK $name: stopped"
      return 0
    fi
    sleep 0.5
  done

  kill -KILL "$pid" >/dev/null 2>&1 || true
  for _ in {1..4}; do
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      rm -f "$pid_file"
      echo "OK $name: stopped"
      return 0
    fi
    sleep 0.5
  done

  echo "FAIL $name: process $pid is still running" >&2
  return 1
}

stop_one fastlio
stop_one mid360
