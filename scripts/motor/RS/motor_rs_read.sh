#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
INTERFACE="${RS_CAN_INTERFACE:-can0}"
MODE="--watch"
REFRESH_MS=100

usage() {
  echo "Usage: $0 [--interface can0] [--refresh-ms 100] [--once]"
  echo "Continuously print RobStride RS motor states without enabling motors."
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --interface)
      [[ $# -ge 2 ]] || { echo "--interface requires a value" >&2; exit 2; }
      INTERFACE=$2
      shift 2
      ;;
    --refresh-ms)
      [[ $# -ge 2 ]] || { echo "--refresh-ms requires a value" >&2; exit 2; }
      REFRESH_MS=$2
      shift 2
      ;;
    --once)
      MODE=""
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ $INTERFACE =~ ^[[:alnum:]_.-]+$ ]] || {
  echo "Invalid SocketCAN interface: $INTERFACE" >&2
  exit 2
}
[[ $REFRESH_MS =~ ^[0-9]+$ ]] || {
  echo "refresh-ms must be an integer from 20 to 60000" >&2
  exit 2
}
(( REFRESH_MS >= 20 && REFRESH_MS <= 60000 )) || {
  echo "refresh-ms must be an integer from 20 to 60000" >&2
  exit 2
}

set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2
  exit 2
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

if [[ -n $MODE ]]; then
  exec ros2 run rs_motor_sdk rs_motor_read_state "${INTERFACE}" "${MODE}" "${REFRESH_MS}"
else
  exec ros2 run rs_motor_sdk rs_motor_read_state "${INTERFACE}"
fi
