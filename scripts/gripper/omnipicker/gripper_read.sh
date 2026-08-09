#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
source "${SCRIPT_DIR}/gripper_common.bash"

INTERFACE="$OMNIPICKER_DEFAULT_INTERFACE"
CAN_ID="$OMNIPICKER_DEFAULT_CAN_ID"
TIMEOUT_MS=1000
REFRESH_MS=100
ONCE=false
DRY_RUN=false

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

validate_omnipicker_interface_name "$INTERFACE"
validate_omnipicker_can_id "$CAN_ID"
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

ensure_omnipicker_can_interface "$INTERFACE"
source_omnipicker_workspace "$WORKSPACE_DIR"

if [[ $ONCE == true ]]; then
  exec "${COMMAND[@]}"
fi

echo "Watching Omnipicker feedback on $INTERFACE id=$CAN_ID; press Ctrl+C to stop."
while true; do
  "${COMMAND[@]}" || true
  sleep "$(awk -v milliseconds="$REFRESH_MS" 'BEGIN { printf "%.3f", milliseconds / 1000.0 }')"
done
