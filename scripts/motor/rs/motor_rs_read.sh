#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
INTERFACE="${RS_CAN_INTERFACE:-can0}"
MODE="--watch"
REFRESH_MS=100
DRY_RUN=false

ensure_rs_can_interface() {
  local interface=$1 bitrate=${RS_CAN_BITRATE:-1000000} restart_ms=${RS_CAN_RESTART_MS:-100} details
  local -a privilege=()
  [[ $bitrate =~ ^[0-9]+$ && $bitrate -gt 0 ]] || { echo "RS_CAN_BITRATE must be a positive integer." >&2; return 2; }
  [[ $restart_ms =~ ^[0-9]+$ ]] || { echo "RS_CAN_RESTART_MS must be a non-negative integer." >&2; return 2; }
  command -v ip >/dev/null 2>&1 || { echo "The 'ip' command is required to configure SocketCAN." >&2; return 2; }
  details=$(ip -details link show dev "$interface" 2>/dev/null) || { echo "SocketCAN interface '$interface' does not exist. Check the CAN adapter and driver." >&2; return 1; }
  [[ $details == *"link/can"* ]] || { echo "Network interface '$interface' is not a CAN interface." >&2; return 1; }
  if [[ $details == *"<"*"UP"*">"* && $details != *"can state STOPPED"* && $details != *"can state BUS-OFF"* ]]; then
    echo "SocketCAN $interface is already active; keeping its current configuration."; return 0
  fi
  if (( EUID != 0 )); then
    command -v sudo >/dev/null 2>&1 || { echo "sudo is required to initialize SocketCAN '$interface'." >&2; return 1; }
    privilege=(sudo)
  fi
  echo "Initializing SocketCAN $interface (classic CAN, ${bitrate} bit/s)..."
  "${privilege[@]}" ip link set dev "$interface" down
  "${privilege[@]}" ip link set dev "$interface" mtu 16
  "${privilege[@]}" ip link set dev "$interface" type can bitrate "$bitrate" restart-ms "$restart_ms"
  "${privilege[@]}" ip link set dev "$interface" up
  details=$(ip -details link show dev "$interface")
  [[ $details == *"<"*"UP"*">"* && $details != *"can state STOPPED"* && $details != *"can state BUS-OFF"* ]] || { echo "SocketCAN '$interface' did not enter an active state." >&2; return 1; }
  echo "SocketCAN $interface is ready."
}

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

ensure_rs_can_interface "$INTERFACE"

set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2
  exit 2
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec "${COMMAND[@]}"
