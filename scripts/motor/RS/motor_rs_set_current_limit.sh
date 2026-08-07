#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
source "${SCRIPT_DIR}/can_common.bash"

INTERFACE="${RS_CAN_INTERFACE:-can0}"
JOINT=""
LIMIT=""
EXECUTE=false
STORE=false
DRY_RUN=false

usage() {
  echo "Usage: $0 --joint joint1 --limit 20 --store --execute [--interface can0]"
  echo "       $0 --joint joint1 --limit 20 --store --dry-run [--interface can0]"
  echo "Persistently change limit_cur for exactly one disabled RS motor."
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --joint) [[ $# -ge 2 ]] || exit 2; JOINT=$2; shift 2 ;;
    --limit) [[ $# -ge 2 ]] || exit 2; LIMIT=$2; shift 2 ;;
    --interface) [[ $# -ge 2 ]] || exit 2; INTERFACE=$2; shift 2 ;;
    --store) STORE=true; shift ;;
    --execute) EXECUTE=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n $JOINT && -n $LIMIT ]] || { usage >&2; exit 2; }
[[ $STORE == true ]] || { echo "--store is required." >&2; exit 2; }
[[ $INTERFACE =~ ^[[:alnum:]_.-]+$ ]] || {
  echo "Invalid SocketCAN interface: $INTERFACE" >&2
  exit 2
}
joint_number=${JOINT,,}
joint_number=${joint_number#joint}
[[ $joint_number =~ ^[1-6]$ ]] || {
  echo "--joint must be exactly one of joint1..joint6." >&2
  exit 2
}
[[ $LIMIT =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || {
  echo "--limit must be a positive number." >&2
  exit 2
}

if (( joint_number <= 3 )); then
  model=RS-06
  maximum=57
else
  model=RS-00
  maximum=16
fi
awk -v value="$LIMIT" -v maximum="$maximum" \
  'BEGIN { exit !(value > 0 && value <= maximum) }' || {
  echo "joint${joint_number} ${model} requires 0 < limit <= ${maximum}." >&2
  exit 2
}

COMMAND=(ros2 run rs_motor_sdk rs_motor_set_current_limit --execute --store \
  "$INTERFACE" "$joint_number" "$LIMIT")
echo "Target: joint${joint_number} model=${model} new_limit=${LIMIT}"
if [[ $DRY_RUN == true ]]; then
  printf 'Dry run; no CAN command was sent: '
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi
[[ $EXECUTE == true ]] || { echo "--execute is required." >&2; exit 2; }

echo "WARNING: this permanently changes the motor current limit."
echo "The target motor will be disabled before the parameter is written."
ensure_rs_can_interface "$INTERFACE"
read -r -p "Type RS_STORE_CURRENT_LIMIT to continue: " confirmation
[[ $confirmation == RS_STORE_CURRENT_LIMIT ]] || { echo "Cancelled."; exit 1; }

set +u
source /opt/ros/humble/setup.bash
[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] || {
  echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2
  exit 2
}
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec "${COMMAND[@]}"
