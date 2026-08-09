#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

POSITION=""
VELOCITY=1.0
FORCE=1.0
INTERFACE="${OMNIPICKER_CAN_INTERFACE:-can0}"
CAN_ID="${OMNIPICKER_CAN_ID:-0x07}"
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

validate_unit_value() {
  local label=$1 value=$2
  [[ $value =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] &&
    awk -v value="$value" 'BEGIN { exit !(value >= 0.0 && value <= 1.0) }' || {
      echo "$label must be a number from 0.0 to 1.0." >&2; return 2;
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
    echo "SocketCAN '$interface' is not active; configure it before controlling the gripper." >&2; return 1;
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
  echo "Usage: $0 --position 0..1 [--velocity 0..1] [--force 0..1]"
  echo "          [--interface can0] [--can-id 0x07] [--dry-run]"
  echo "Send an Omnipicker target. Position 0 is closed and 1 is open."
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --position) [[ $# -ge 2 ]] || { echo "--position requires a value" >&2; exit 2; }; POSITION=$2; shift 2 ;;
    --velocity) [[ $# -ge 2 ]] || { echo "--velocity requires a value" >&2; exit 2; }; VELOCITY=$2; shift 2 ;;
    --force) [[ $# -ge 2 ]] || { echo "--force requires a value" >&2; exit 2; }; FORCE=$2; shift 2 ;;
    --interface) [[ $# -ge 2 ]] || { echo "--interface requires a value" >&2; exit 2; }; INTERFACE=$2; shift 2 ;;
    --can-id) [[ $# -ge 2 ]] || { echo "--can-id requires a value" >&2; exit 2; }; CAN_ID=$2; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n $POSITION ]] || { echo "--position is required." >&2; usage >&2; exit 2; }
validate_unit_value position "$POSITION"
validate_unit_value velocity "$VELOCITY"
validate_unit_value force "$FORCE"
validate_interface_name "$INTERFACE"
validate_can_id "$CAN_ID"

COMMAND=(ros2 run rebotarm_gripper_sdk gripper_command
  "$POSITION" "$VELOCITY" "$FORCE" "$INTERFACE" "$CAN_ID")

printf 'Omnipicker target: position=%s velocity=%s force=%s interface=%s can_id=%s\n' \
  "$POSITION" "$VELOCITY" "$FORCE" "$INTERFACE" "$CAN_ID"
if [[ $DRY_RUN == true ]]; then
  printf 'Dry run; no CAN command was sent: '
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

ensure_can_interface "$INTERFACE"
source_workspace
exec "${COMMAND[@]}"
