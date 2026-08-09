#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
INTERFACE="${RS_CAN_INTERFACE:-can0}"; DURATION="${RS_HOME_DURATION:-5}"; MODE=""; VALUE=""; DRY_RUN=false; VERBOSE=false; HOLD=false
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
usage(){ echo "Usage: $0 --joint joint3|--joints joint1,joint3|--all [--interface can0] [--duration 5] [--verbose] [--hold] [--dry-run]"; echo "Move selected RS joints to calibrated zero (0 rad) with a minimum-jerk trajectory."; }
set_mode(){ [[ -z $MODE ]] || { echo "Choose exactly one selection option." >&2; exit 2; }; MODE=$1; VALUE=${2:-}; }
while [[ $# -gt 0 ]]; do case $1 in
  --joint) [[ $# -ge 2 ]] || exit 2; set_mode joint "$2"; shift 2;;
  --joints) [[ $# -ge 2 ]] || exit 2; set_mode joints "$2"; shift 2;;
  -all|--all) set_mode all; shift;;
  --interface) [[ $# -ge 2 ]] || exit 2; INTERFACE=$2; shift 2;;
  --duration) [[ $# -ge 2 ]] || { echo "--duration requires a value" >&2; exit 2; }; DURATION=$2; shift 2;;
  --verbose) VERBOSE=true; shift;;
  --hold) HOLD=true; shift;;
  --dry-run) DRY_RUN=true; shift;;
  --execute) shift;; # Kept for compatibility; execution is now the default.
  -h|--help) usage; exit 0;; *) echo "Unknown argument: $1" >&2; usage >&2; exit 2;; esac; done
[[ -n $MODE ]] || { usage >&2; echo "One selection is required." >&2; exit 2; }
[[ $INTERFACE =~ ^[[:alnum:]_.-]+$ ]] || { echo "Invalid SocketCAN interface: $INTERFACE" >&2; exit 2; }
[[ $DURATION =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || { echo "duration must be from 1 to 120 seconds" >&2; exit 2; }
awk -v duration="$DURATION" 'BEGIN { exit !(duration >= 1 && duration <= 120) }' || { echo "duration must be from 1 to 120 seconds" >&2; exit 2; }
normalize(){ local v=${1,,}; v=${v#joint}; [[ $v =~ ^[1-6]$ ]] || { echo "Invalid joint '$1'." >&2; exit 2; }; printf '%s' "$v"; }
IDS=(); case $MODE in joint) IDS+=("$(normalize "$VALUE")");; joints) IFS=',' read -r -a a <<< "$VALUE"; for v in "${a[@]}"; do IDS+=("$(normalize "$v")"); done;; all) IDS=(1 2 3 4 5 6);; esac
mapfile -t IDS < <(printf '%s\n' "${IDS[@]}" | sort -n -u)
echo "WARNING: selected joints will move to calibrated zero: $(printf 'joint%s ' "${IDS[@]}")"
echo "Ensure the arm has clearance and can be stopped immediately."
COMMAND=(ros2 run rs_motor_sdk rs_motor_home --execute "$INTERFACE" "${IDS[@]}" --duration "$DURATION")
[[ $VERBOSE == true ]] && COMMAND+=(--verbose)
[[ $HOLD == true ]] && COMMAND+=(--hold)
if [[ $DRY_RUN == true ]]; then
  printf 'Dry run; no CAN command was sent: '; printf '%q ' "${COMMAND[@]}"; printf '\n'
  exit 0
fi
ensure_rs_can_interface "$INTERFACE"
read -r -p "Type RS_HOME to continue: " confirm; [[ $confirm == RS_HOME ]] || { echo Cancelled.; exit 1; }
set +u
source /opt/ros/humble/setup.bash
[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] || { echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2; exit 2; }
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
exec "${COMMAND[@]}"
