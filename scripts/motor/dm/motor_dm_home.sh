#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
INTERFACE="${DM_CAN_INTERFACE:-can0}"
DRY_RUN=false

ensure_dm_can_interface() {
  local interface=$1 bitrate=${DM_CAN_BITRATE:-1000000} restart_ms=${DM_CAN_RESTART_MS:-0} details
  local -a privilege=()

  [[ $bitrate =~ ^[0-9]+$ && $bitrate -gt 0 ]] || {
    echo "DM_CAN_BITRATE must be a positive integer." >&2
    return 2
  }
  [[ $restart_ms =~ ^[0-9]+$ ]] || {
    echo "DM_CAN_RESTART_MS must be a non-negative integer." >&2
    return 2
  }
  command -v ip >/dev/null 2>&1 || {
    echo "The 'ip' command is required to configure SocketCAN." >&2
    return 2
  }
  details=$(ip -details link show dev "$interface" 2>/dev/null) || {
    echo "SocketCAN interface '$interface' does not exist. Check the CAN adapter and driver." >&2
    return 1
  }
  [[ $details == *"link/can"* ]] || {
    echo "Network interface '$interface' is not a CAN interface." >&2
    return 1
  }
  if [[ $details == *"<"*"UP"*">"* &&
        $details != *"can state STOPPED"* &&
        $details != *"can state BUS-OFF"* ]]; then
    echo "SocketCAN $interface is already active; keeping its current configuration."
    return 0
  fi
  if (( EUID != 0 )); then
    command -v sudo >/dev/null 2>&1 || {
      echo "sudo is required to initialize SocketCAN '$interface'." >&2
      return 1
    }
    privilege=(sudo)
  fi
  echo "Initializing SocketCAN $interface (classic CAN, ${bitrate} bit/s)..."
  "${privilege[@]}" ip link set dev "$interface" down
  "${privilege[@]}" ip link set dev "$interface" mtu 16
  if (( restart_ms > 0 )); then
    if ! "${privilege[@]}" ip link set dev "$interface" type can bitrate "$bitrate" restart-ms "$restart_ms"; then
      echo "SocketCAN $interface does not support automatic Bus-Off restart; continuing without it." >&2
      "${privilege[@]}" ip link set dev "$interface" type can bitrate "$bitrate"
    fi
  else
    "${privilege[@]}" ip link set dev "$interface" type can bitrate "$bitrate"
  fi
  "${privilege[@]}" ip link set dev "$interface" up
  details=$(ip -details link show dev "$interface")
  [[ $details == *"<"*"UP"*">"* &&
        $details != *"can state STOPPED"* &&
        $details != *"can state BUS-OFF"* ]] || {
    echo "SocketCAN '$interface' did not enter an active state." >&2
    return 1
  }
  echo "SocketCAN $interface is ready."
}

usage() {
  echo "Usage: $0 [--interface can0] [--dry-run]"
  echo "Move all six DM arm joints to motor-coordinate zero with minimum jerk."
  echo "The underlying tool selects joint1..joint6 as one safety-checked group."
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --interface)
      [[ $# -ge 2 ]] || { echo "--interface requires a value." >&2; exit 2; }
      INTERFACE=$2
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --execute)
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

echo "WARNING: all six DM arm joints will move to motor-coordinate zero."
echo "Ensure the arm has clearance and can be stopped immediately."
COMMAND=(ros2 run rebotarm_dm_motor_sdk dm_motor_minimum_jerk_home --execute "$INTERFACE")
if [[ $DRY_RUN == true ]]; then
  printf 'Dry run; no CAN command was sent: '
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

ensure_dm_can_interface "$INTERFACE"
read -r -p "Type DM_HOME to continue: " confirmation
[[ $confirmation == DM_HOME ]] || { echo "Cancelled."; exit 1; }

set +u
source /opt/ros/humble/setup.bash
[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] || {
  echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2
  exit 2
}
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec "${COMMAND[@]}"
