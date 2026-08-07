#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
INTERFACE="${RS_CAN_INTERFACE:-can0}"
DRY_RUN=false
SELECTION_MODE=""
SELECTION_VALUE=""

usage() {
  echo "Usage:"
  echo "  $0 --joint joint3 [--interface can0] [--dry-run]"
  echo "  $0 --joints joint1,joint3,joint6 [--interface can0] [--dry-run]"
  echo "  $0 --all [--interface can0] [--dry-run]"
  echo "Disable selected RS motors, clear their faults, and verify feedback."
}

set_selection() {
  [[ -z $SELECTION_MODE ]] || {
    echo "Choose exactly one of --joint, --joints, or --all." >&2
    exit 2
  }
  SELECTION_MODE=$1
  SELECTION_VALUE=${2:-}
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --joint) [[ $# -ge 2 ]] || exit 2; set_selection joint "$2"; shift 2 ;;
    --joints) [[ $# -ge 2 ]] || exit 2; set_selection joints "$2"; shift 2 ;;
    --all) set_selection all; shift ;;
    --interface) [[ $# -ge 2 ]] || exit 2; INTERFACE=$2; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    --execute) shift ;; # Kept for compatibility; execution is now the default.
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n $SELECTION_MODE ]] || {
  echo "One joint selection is required." >&2
  usage >&2
  exit 2
}
[[ $INTERFACE =~ ^[[:alnum:]_.-]+$ ]] || {
  echo "Invalid SocketCAN interface: $INTERFACE" >&2
  exit 2
}

normalize_joint() {
  local value=${1,,}
  value=${value#joint}
  [[ $value =~ ^[1-6]$ ]] || {
    echo "Invalid joint '$1'; expected joint1..joint6 or 1..6." >&2
    exit 2
  }
  printf '%s\n' "$value"
}

JOINT_IDS=()
case $SELECTION_MODE in
  joint) JOINT_IDS+=("$(normalize_joint "$SELECTION_VALUE")") ;;
  joints)
    IFS=',' read -r -a VALUES <<< "$SELECTION_VALUE"
    for value in "${VALUES[@]}"; do
      JOINT_IDS+=("$(normalize_joint "$value")")
    done
    ;;
  all) JOINT_IDS=(1 2 3 4 5 6) ;;
esac
mapfile -t JOINT_IDS < <(printf '%s\n' "${JOINT_IDS[@]}" | sort -n -u)

echo "Selected motors will be disabled before clearing faults: $(printf 'joint%s ' "${JOINT_IDS[@]}")"
COMMAND=(ros2 run rs_motor_sdk rs_motor_clear_fault --execute "${INTERFACE}" "${JOINT_IDS[@]}")
if [[ $DRY_RUN == true ]]; then
  printf 'Dry run; no CAN command was sent: '
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi
read -r -p "Type RS_CLEAR to continue: " confirmation
[[ $confirmation == RS_CLEAR ]] || { echo "Cancelled."; exit 1; }

set +u
source /opt/ros/humble/setup.bash
[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] || {
  echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2
  exit 2
}
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec "${COMMAND[@]}"
