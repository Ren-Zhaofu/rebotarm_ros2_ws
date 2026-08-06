#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=motor_common.sh
source "${SCRIPT_DIR}/motor_common.sh"

usage() {
    echo "Usage: $0 [--interface can0]"
    echo "Display the selected DM SocketCAN interface without changing it."
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
    usage
    exit 0
fi

dm_resolve_interface "$@"
if [[ ${#DM_MOTOR_ARGS[@]} -ne 0 ]]; then
    usage >&2
    exit 2
fi
dm_require_interface

echo "DM motor transport: SocketCAN"
echo "SocketCAN interface: ${DM_MOTOR_INTERFACE}"
ip -details link show "${DM_MOTOR_INTERFACE}"
