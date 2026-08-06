#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=motor_common.sh
source "${SCRIPT_DIR}/motor_common.sh"

usage() {
    echo "Usage: $0 --execute [--interface can0]"
    echo "Move the six DM arm joints to motor coordinate zero."
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
    usage
    exit 0
fi
if [[ ${1:-} != "--execute" ]]; then
    echo "Refusing motion: --execute is required." >&2
    usage >&2
    exit 2
fi
shift

dm_resolve_interface "$@"
if [[ ${#DM_MOTOR_ARGS[@]} -ne 0 ]]; then
    echo "Unexpected argument: ${DM_MOTOR_ARGS[0]}" >&2
    exit 2
fi
dm_require_interface
dm_source_workspace
set -u

cat <<EOF
WARNING: This command will enable and move six DM arm joints.
Interface: ${DM_MOTOR_INTERFACE}
Keep clear of the robot and keep hardware emergency stop available.
EOF
read -r -p "Type DM_HOME to continue: " confirmation
if [[ $confirmation != "DM_HOME" ]]; then
    echo "Cancelled."
    exit 1
fi

exec ros2 run rebotarm_dm_motor_sdk dm_motor_minimum_jerk_home \
    --execute "${DM_MOTOR_INTERFACE}"
