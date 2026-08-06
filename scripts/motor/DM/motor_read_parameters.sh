#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=motor_common.sh
source "${SCRIPT_DIR}/motor_common.sh"

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
    echo "Usage: $0 [--interface can0]"
    echo "Read DM motor parameters without enabling or moving motors."
    exit 0
fi

dm_resolve_interface "$@"
if [[ ${#DM_MOTOR_ARGS[@]} -ne 0 ]]; then
    echo "Unexpected argument: ${DM_MOTOR_ARGS[0]}" >&2
    exit 2
fi
dm_require_interface
dm_source_workspace
set -u

exec ros2 run rebotarm_dm_motor_sdk dm_motor_read_parameters \
    "${DM_MOTOR_INTERFACE}"
