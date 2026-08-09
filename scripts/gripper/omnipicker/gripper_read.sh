#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

INTERFACE="${OMNIPICKER_CAN_INTERFACE:-can0}"
CAN_ID="${OMNIPICKER_CAN_ID:-0x07}"
TIMEOUT_MS=1000
REFRESH_MS=100
ONCE=false
DRY_RUN=false

validate_interface_name() {
  [[ $1 =~ ^[[:alnum:]_.-]+$ ]] || {
    echo "Invalid SocketCAN interface: $1" >&2; return 2;
  }
}

validate_can_id() {
  local can_id=$1 numeric_id
  [[ $can_id =~ ^(0[xX][0-9a-fA-F]+|[0-9]+)$ ]] || {
    echo "Invalid CAN ID '$can_id'; expected an integer from 0 to 0x7FF." >&2; return 2;
  }
  numeric_id=$((can_id))
  (( numeric_id >= 0 && numeric_id <= 0x7FF )) || {
    echo "Invalid CAN ID '$can_id'; expected an integer from 0 to 0x7FF." >&2; return 2;
  }
}

ensure_can_interface() {
  local interface=$1 details
  command -v ip >/dev/null 2>&1 || {
    echo "The 'ip' command is required to inspect SocketCAN." >&2; return 2;
  }
  details=$(ip -details link show dev "$interface" 2>/dev/null) || {
    echo "SocketCAN interface '$interface' does not exist. Check the CAN adapter and driver." >&2; return 1;
  }
  [[ $details == *"link/can"* ]] || {
    echo "Network interface '$interface' is not a CAN interface." >&2; return 1;
  }
  [[ $details == *"<"*"UP"*">"* && $details != *"can state STOPPED"* && $details != *"can state BUS-OFF"* ]] || {
    echo "SocketCAN '$interface' is not active; configure it before reading the gripper." >&2; return 1;
  }
  echo "SocketCAN $interface is active; keeping its current configuration."
}

source_workspace() {
  set +u
  source /opt/ros/humble/setup.bash
  if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2; return 2
  fi
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
}

usage() {
  echo "Usage: $0 [--interface can0] [--can-id 0x07] [--timeout-ms 1000]"
  echo "          [--refresh-ms 100] [--once] [--dry-run]"
  echo "Continuously read Omnipicker feedback without sending control frames."
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --interface) [[ $# -ge 2 ]] || { echo "--interface requires a value" >&2; exit 2; }; INTERFACE=$2; shift 2 ;;
    --can-id) [[ $# -ge 2 ]] || { echo "--can-id requires a value" >&2; exit 2; }; CAN_ID=$2; shift 2 ;;
    --timeout-ms) [[ $# -ge 2 ]] || { echo "--timeout-ms requires a value" >&2; exit 2; }; TIMEOUT_MS=$2; shift 2 ;;
    --refresh-ms) [[ $# -ge 2 ]] || { echo "--refresh-ms requires a value" >&2; exit 2; }; REFRESH_MS=$2; shift 2 ;;
    --once) ONCE=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

validate_interface_name "$INTERFACE"
validate_can_id "$CAN_ID"
[[ $TIMEOUT_MS =~ ^[0-9]+$ ]] && (( TIMEOUT_MS >= 1 && TIMEOUT_MS <= 60000 )) || {
  echo "timeout-ms must be an integer from 1 to 60000." >&2
  exit 2
}
[[ $REFRESH_MS =~ ^[0-9]+$ ]] && (( REFRESH_MS >= 20 && REFRESH_MS <= 60000 )) || {
  echo "refresh-ms must be an integer from 20 to 60000." >&2
  exit 2
}

COMMAND=(ros2 run rebotarm_gripper_sdk gripper_read_state
  "$INTERFACE" "$CAN_ID" "$TIMEOUT_MS")
if [[ $DRY_RUN == true ]]; then
  printf 'Dry run; no CAN frame was read: '
  printf '%q ' "${COMMAND[@]}"
  [[ $ONCE == true ]] || printf '(repeat every %s ms) ' "$REFRESH_MS"
  printf '\n'
  exit 0
fi

ensure_can_interface "$INTERFACE"
source_workspace

if [[ $ONCE == true ]]; then
  exec "${COMMAND[@]}"
fi

echo "Watching Omnipicker feedback on $INTERFACE id=$CAN_ID; press Ctrl+C to stop."
while true; do
  "${COMMAND[@]}" || true
  sleep "$(awk -v milliseconds="$REFRESH_MS" 'BEGIN { printf "%.3f", milliseconds / 1000.0 }')"
done
