#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
INTERFACE="${RS_CAN_INTERFACE:-can0}"
DRY_RUN=false
SELECTION_MODE=""
SELECTION_VALUE=""

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
  echo "Usage:"
  echo "  $0 --joint joint3 [--interface can0] [--dry-run]"
  echo "  $0 --joints joint1,joint3,joint6 [--interface can0] [--dry-run]"
  echo "  $0 --all [--interface can0] [--dry-run]"
  echo "Set the current mechanical position as zero for selected RS motors."
}

set_selection() {
  if [[ -n $SELECTION_MODE ]]; then
    echo "Choose exactly one of --joint, --joints, or --all." >&2
    exit 2
  fi
  SELECTION_MODE=$1
  SELECTION_VALUE=${2:-}
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --joint)
      [[ $# -ge 2 ]] || { echo "--joint requires a value" >&2; exit 2; }
      set_selection joint "$2"
      shift 2
      ;;
    --joints)
      [[ $# -ge 2 ]] || { echo "--joints requires a value" >&2; exit 2; }
      set_selection joints "$2"
      shift 2
      ;;
    --all)
      set_selection all
      shift
      ;;
    --interface)
      [[ $# -ge 2 ]] || { echo "--interface requires a value" >&2; exit 2; }
      INTERFACE=$2
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --execute)
      shift # Kept for compatibility; execution is now the default.
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

[[ -n $SELECTION_MODE ]] || { usage >&2; exit 2; }
[[ $INTERFACE =~ ^[[:alnum:]_.-]+$ ]] || {
  echo "Invalid SocketCAN interface: $INTERFACE" >&2
  exit 2
}

declare -a JOINT_IDS=()
normalize_joint() {
  local value=${1,,}
  value=${value#joint}
  [[ $value =~ ^[1-6]$ ]] || {
    echo "Invalid joint '$1'; expected joint1..joint6 or 1..6." >&2
    exit 2
  }
  printf '%s\n' "$value"
}

case $SELECTION_MODE in
  joint)
    JOINT_IDS+=("$(normalize_joint "$SELECTION_VALUE")")
    ;;
  joints)
    IFS=',' read -r -a VALUES <<< "$SELECTION_VALUE"
    [[ ${#VALUES[@]} -gt 0 ]] || { echo "--joints is empty" >&2; exit 2; }
    for value in "${VALUES[@]}"; do
      JOINT_IDS+=("$(normalize_joint "$value")")
    done
    ;;
  all)
    JOINT_IDS=(1 2 3 4 5 6)
    ;;
esac

mapfile -t JOINT_IDS < <(printf '%s\n' "${JOINT_IDS[@]}" | sort -n -u)
JOINT_LABELS=()
for id in "${JOINT_IDS[@]}"; do
  JOINT_LABELS+=("joint${id}")
done

echo "WARNING: This permanently changes the software zero of: ${JOINT_LABELS[*]}"
echo "The selected joints must already be positioned at their intended URDF zero."
echo "The command will disable the selected motors before writing zero."
COMMAND=(ros2 run rs_motor_sdk rs_motor_set_zero --execute "${INTERFACE}" "${JOINT_IDS[@]}")
if [[ $DRY_RUN == true ]]; then
  printf 'Dry run; no CAN command was sent: '
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi
ensure_rs_can_interface "$INTERFACE"
read -r -p "Type RS_ZERO to continue: " confirmation
[[ $confirmation == RS_ZERO ]] || { echo "Cancelled."; exit 1; }

set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2
  exit 2
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec "${COMMAND[@]}"
