#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IFACE="${IFACE:-${1:-enp2s0}}"
HOST_CIDR="${HOST_CIDR:-${2:-192.168.1.5/24}}"

"$SCRIPT_DIR/mid360.sh" "$IFACE" "$HOST_CIDR"
"$SCRIPT_DIR/fastlio.sh"

echo "OK run"
