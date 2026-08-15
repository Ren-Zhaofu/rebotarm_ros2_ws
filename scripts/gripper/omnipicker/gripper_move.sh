#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

POSITION=""
VELOCITY=1.0
FORCE=1.0
CYCLE=false
CYCLES=3
INTERVAL_MS=1000
INTERFACE="${OMNIPICKER_CAN_INTERFACE:-can0}"
CAN_ID="${OMNIPICKER_CAN_ID:-0x07}"
DURATION_MS="${OMNIPICKER_COMMAND_DURATION_MS:-1000}"
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
  local interface=$1
  local bitrate=${OMNIPICKER_CAN_BITRATE:-1000000}
  local restart_ms=${OMNIPICKER_CAN_RESTART_MS:-100}
  local details
  local -a privilege=()

  [[ $bitrate =~ ^[0-9]+$ && $bitrate -gt 0 ]] || {
    echo "OMNIPICKER_CAN_BITRATE must be a positive integer." >&2; return 2;
  }
  [[ $restart_ms =~ ^[0-9]+$ ]] || {
    echo "OMNIPICKER_CAN_RESTART_MS must be a non-negative integer." >&2; return 2;
  }
  command -v ip >/dev/null 2>&1 || {
    echo "The 'ip' command is required to configure SocketCAN." >&2; return 2;
  }
  details=$(ip -details link show dev "$interface" 2>/dev/null) || {
    echo "SocketCAN interface '$interface' does not exist. Check the CAN adapter and driver." >&2; return 1;
  }
  [[ $details == *"link/can"* ]] || {
    echo "Network interface '$interface' is not a CAN interface." >&2; return 1;
  }
  if [[ $details == *"<"*"UP"*">"* &&
       $details != *"can state STOPPED"* &&
       $details != *"can state BUS-OFF"* ]]; then
    echo "SocketCAN $interface is active; keeping its current configuration."
    return 0
  fi
  if (( EUID != 0 )); then
    command -v sudo >/dev/null 2>&1 || {
      echo "sudo is required to initialize SocketCAN '$interface'." >&2; return 1;
    }
    privilege=(sudo)
  fi
  echo "Initializing SocketCAN $interface (classic CAN, ${bitrate} bit/s)..."
  "${privilege[@]}" ip link set dev "$interface" down
  "${privilege[@]}" ip link set dev "$interface" mtu 16
  if (( restart_ms > 0 )); then
    if ! "${privilege[@]}" ip link set dev "$interface" type can \
      bitrate "$bitrate" restart-ms "$restart_ms"; then
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
    echo "SocketCAN '$interface' did not enter an active state." >&2; return 1;
  }
  echo "SocketCAN $interface is ready."
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
  echo "Usage: $0 --position 0..1 [options]"
  echo "       $0 --cycle [--cycles 3] [--interval-ms 1000] [options]"
  echo "Options: --velocity 0..1 --force 0..1 --interface can0 --can-id 0x07"
  echo "         --duration-ms 1000 --dry-run"
  echo "Send an Omnipicker target. Position 0 is closed and 1 is open."
  echo "Cycle mode alternates open and closed; --cycles 0 runs until interrupted."
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --position) [[ $# -ge 2 ]] || { echo "--position requires a value" >&2; exit 2; }; POSITION=$2; shift 2 ;;
    --cycle) CYCLE=true; shift ;;
    --cycles) [[ $# -ge 2 ]] || { echo "--cycles requires a value" >&2; exit 2; }; CYCLES=$2; shift 2 ;;
    --interval-ms) [[ $# -ge 2 ]] || { echo "--interval-ms requires a value" >&2; exit 2; }; INTERVAL_MS=$2; shift 2 ;;
    --velocity) [[ $# -ge 2 ]] || { echo "--velocity requires a value" >&2; exit 2; }; VELOCITY=$2; shift 2 ;;
    --force) [[ $# -ge 2 ]] || { echo "--force requires a value" >&2; exit 2; }; FORCE=$2; shift 2 ;;
    --interface) [[ $# -ge 2 ]] || { echo "--interface requires a value" >&2; exit 2; }; INTERFACE=$2; shift 2 ;;
    --can-id) [[ $# -ge 2 ]] || { echo "--can-id requires a value" >&2; exit 2; }; CAN_ID=$2; shift 2 ;;
    --duration-ms) [[ $# -ge 2 ]] || { echo "--duration-ms requires a value" >&2; exit 2; }; DURATION_MS=$2; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ $CYCLE == true ]]; then
  [[ -z $POSITION ]] || {
    echo "--position cannot be combined with --cycle." >&2
    exit 2
  }
else
  [[ -n $POSITION ]] || { echo "--position or --cycle is required." >&2; usage >&2; exit 2; }
  validate_unit_value position "$POSITION"
fi
validate_unit_value velocity "$VELOCITY"
validate_unit_value force "$FORCE"
validate_interface_name "$INTERFACE"
validate_can_id "$CAN_ID"
[[ $DURATION_MS =~ ^[0-9]+$ ]] && (( DURATION_MS >= 100 && DURATION_MS <= 60000 )) || {
  echo "duration-ms must be an integer from 100 to 60000." >&2
  exit 2
}
[[ $CYCLES =~ ^[0-9]+$ ]] && (( CYCLES <= 10000 )) || {
  echo "cycles must be an integer from 0 to 10000; 0 means continuous." >&2
  exit 2
}
[[ $INTERVAL_MS =~ ^[0-9]+$ ]] && (( INTERVAL_MS <= 60000 )) || {
  echo "interval-ms must be an integer from 0 to 60000." >&2
  exit 2
}

run_target() {
  local position=$1
  local -a command=(ros2 run rebotarm_gripper_sdk gripper_command
    "$position" "$VELOCITY" "$FORCE" "$INTERFACE" "$CAN_ID" "$DURATION_MS")

  printf 'Omnipicker target: position=%s velocity=%s force=%s interface=%s can_id=%s duration=%s ms\n' \
    "$position" "$VELOCITY" "$FORCE" "$INTERFACE" "$CAN_ID" "$DURATION_MS"
  if [[ $DRY_RUN == true ]]; then
    printf 'Dry run; no CAN command was sent: '
    printf '%q ' "${command[@]}"
    printf '\n'
    return
  fi
  "${command[@]}"
}

if [[ $DRY_RUN == true ]]; then
  if [[ $CYCLE == true ]]; then
    printf 'Cycle preview: cycles=%s interval=%s ms (open, then close)\n' "$CYCLES" "$INTERVAL_MS"
    run_target 1.0
    run_target 0.0
  else
    run_target "$POSITION"
  fi
  exit 0
fi

ensure_can_interface "$INTERFACE"
source_workspace

if [[ $CYCLE == false ]]; then
  run_target "$POSITION"
  exit 0
fi

sleep_seconds=$(awk -v milliseconds="$INTERVAL_MS" 'BEGIN { printf "%.3f", milliseconds / 1000 }')
trap 'printf "\nCycle stopped.\n"; exit 130' INT TERM
printf 'Cycle mode: cycles=%s interval=%s ms. Press Ctrl+C to stop.\n' "$CYCLES" "$INTERVAL_MS"

completed=0
while (( CYCLES == 0 || completed < CYCLES )); do
  printf 'Cycle %d: opening.\n' "$((completed + 1))"
  run_target 1.0
  if (( INTERVAL_MS > 0 )); then
    sleep "$sleep_seconds"
  fi

  printf 'Cycle %d: closing.\n' "$((completed + 1))"
  run_target 0.0
  completed=$((completed + 1))
  if (( INTERVAL_MS > 0 && (CYCLES == 0 || completed < CYCLES) )); then
    sleep "$sleep_seconds"
  fi
done
