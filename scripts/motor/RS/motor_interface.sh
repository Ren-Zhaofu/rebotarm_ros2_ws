#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=motor_common.sh
source "${SCRIPT_DIR}/motor_common.sh"

usage() {
    echo "Usage: $0 [--interface enp3s0]"
    echo "Detect and display the RS EtherCAT interface without opening it."
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
    usage
    exit 0
fi

rs_resolve_interface "$@"
if [[ ${#RS_MOTOR_ARGS[@]} -ne 0 ]]; then
    usage >&2
    exit 2
fi

echo "RS motor transport: EYou EtherCAT"
echo "EtherCAT interface: ${RS_MOTOR_INTERFACE}"
ip -details link show "${RS_MOTOR_INTERFACE}"
if [[ -n ${EYOU_SDK_LIB:-} ]]; then
    echo "EYou SDK library directory: ${EYOU_SDK_LIB}"
else
    echo "EYou SDK library directory: not configured"
fi
