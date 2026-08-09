#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
source "${SCRIPT_DIR}/gripper_common.bash"

POSITION=""
VELOCITY=1.0
FORCE=1.0
INTERFACE="$OMNIPICKER_DEFAULT_INTERFACE"
CAN_ID="$OMNIPICKER_DEFAULT_CAN_ID"
DRY_RUN=false

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
validate_omnipicker_unit_value position "$POSITION"
validate_omnipicker_unit_value velocity "$VELOCITY"
validate_omnipicker_unit_value force "$FORCE"
validate_omnipicker_interface_name "$INTERFACE"
validate_omnipicker_can_id "$CAN_ID"

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

ensure_omnipicker_can_interface "$INTERFACE"
source_omnipicker_workspace "$WORKSPACE_DIR"
exec "${COMMAND[@]}"
