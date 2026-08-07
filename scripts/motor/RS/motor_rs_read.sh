#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
INTERFACE="${RS_CAN_INTERFACE:-can0}"
MODE="--watch"
REFRESH_MS=100
DRY_RUN=false

usage() {
  echo "Usage: $0 [--interface can0] [--refresh-ms 100] [--once] [--dry-run]"
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
    --dry-run)
      DRY_RUN=true
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

COMMAND=(ros2 run rs_motor_sdk rs_motor_read_state "${INTERFACE}")
if [[ -n $MODE ]]; then
  COMMAND+=("${MODE}" "${REFRESH_MS}")
fi
if [[ $DRY_RUN == true ]]; then
  printf 'Dry run; no CAN command was sent: '
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2
  exit 2
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec "${COMMAND[@]}"
