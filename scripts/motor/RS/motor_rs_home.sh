#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
INTERFACE="${RS_CAN_INTERFACE:-can0}"; MODE=""; VALUE=""; EXECUTE=false
usage(){ echo "Usage: $0 --joint joint3|--joints joint1,joint3|--all --execute [--interface can0]"; echo "Move selected RS joints gradually to calibrated zero (0 rad)."; }
set_mode(){ [[ -z $MODE ]] || { echo "Choose exactly one selection option." >&2; exit 2; }; MODE=$1; VALUE=${2:-}; }
while [[ $# -gt 0 ]]; do case $1 in
  --joint) [[ $# -ge 2 ]] || exit 2; set_mode joint "$2"; shift 2;;
  --joints) [[ $# -ge 2 ]] || exit 2; set_mode joints "$2"; shift 2;;
  --all) set_mode all; shift;;
  --interface) [[ $# -ge 2 ]] || exit 2; INTERFACE=$2; shift 2;;
  --execute) EXECUTE=true; shift;; -h|--help) usage; exit 0;; *) echo "Unknown argument: $1" >&2; usage >&2; exit 2;; esac; done
[[ -n $MODE && $EXECUTE == true ]] || { usage >&2; echo "--execute and one selection are required." >&2; exit 2; }
[[ $INTERFACE =~ ^[[:alnum:]_.-]+$ ]] || { echo "Invalid SocketCAN interface: $INTERFACE" >&2; exit 2; }
normalize(){ local v=${1,,}; v=${v#joint}; [[ $v =~ ^[1-6]$ ]] || { echo "Invalid joint '$1'." >&2; exit 2; }; printf '%s' "$v"; }
IDS=(); case $MODE in joint) IDS+=("$(normalize "$VALUE")");; joints) IFS=',' read -r -a a <<< "$VALUE"; for v in "${a[@]}"; do IDS+=("$(normalize "$v")"); done;; all) IDS=(1 2 3 4 5 6);; esac
mapfile -t IDS < <(printf '%s\n' "${IDS[@]}" | sort -n -u)
echo "WARNING: selected joints will move to calibrated zero: $(printf 'joint%s ' "${IDS[@]}")"
echo "Ensure the arm has clearance and can be stopped immediately."
read -r -p "Type RS_HOME to continue: " confirm; [[ $confirm == RS_HOME ]] || { echo Cancelled.; exit 1; }
set +u
source /opt/ros/humble/setup.bash
[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] || { echo "Workspace is not built. Run ${WORKSPACE_DIR}/scripts/build.sh first." >&2; exit 2; }
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
exec ros2 run rs_motor_sdk rs_motor_home --execute "$INTERFACE" "${IDS[@]}"
