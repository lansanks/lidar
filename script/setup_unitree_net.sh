#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-enp2s0}"
HOST_CIDR="${2:-192.168.1.2/24}"
LIDAR_IP="${3:-192.168.1.62}"

if ! command -v ip >/dev/null 2>&1; then
  echo "Error: ip command not found. Please install iproute2." >&2
  exit 1
fi

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "Error: network interface '$IFACE' does not exist." >&2
  exit 1
fi

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

echo "Configuring Unitree L2 network interface: $IFACE -> $HOST_CIDR"
echo "Expected lidar IP: $LIDAR_IP"
"${SUDO[@]}" ip addr flush dev "$IFACE"
"${SUDO[@]}" ip addr add "$HOST_CIDR" dev "$IFACE"
"${SUDO[@]}" ip link set "$IFACE" up

echo "Done. Current address on $IFACE:"
ip -brief addr show "$IFACE"

echo
echo "ROS2 driver launch should use:"
echo "  local_ip: ${HOST_CIDR%/*}"
echo "  lidar_ip: $LIDAR_IP"
