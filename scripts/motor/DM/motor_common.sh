#!/usr/bin/env bash

DM_MOTOR_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=motor_config.sh
source "${DM_MOTOR_SCRIPT_DIR}/motor_config.sh"

dm_resolve_interface() {
    local interface=${DM_CAN_INTERFACE:-can0}
    DM_MOTOR_ARGS=("$@")

    if [[ ${DM_MOTOR_ARGS[0]:-} == "--interface" ]]; then
        if [[ -z ${DM_MOTOR_ARGS[1]:-} ]]; then
            echo "--interface requires a SocketCAN interface name." >&2
            return 2
        fi
        interface=${DM_MOTOR_ARGS[1]}
        DM_MOTOR_ARGS=("${DM_MOTOR_ARGS[@]:2}")
    fi

    if [[ -z $interface ]]; then
        echo "DM_CAN_INTERFACE is empty in ${DM_MOTOR_SCRIPT_DIR}/motor_config.sh." >&2
        return 2
    fi
    DM_MOTOR_INTERFACE=$interface
}

dm_require_interface() {
    if [[ ! -d /sys/class/net/${DM_MOTOR_INTERFACE} ]]; then
        echo "SocketCAN interface '${DM_MOTOR_INTERFACE}' does not exist." >&2
        echo "Connect the CAN adapter and configure the interface first." >&2
        return 2
    fi
}

dm_source_workspace() {
    local workspace_dir
    workspace_dir="$(cd -- "${DM_MOTOR_SCRIPT_DIR}/../../.." && pwd)"
    source /opt/ros/humble/setup.bash
    if [[ ! -f ${workspace_dir}/install/setup.bash ]]; then
        echo "Workspace is not built. Run ${workspace_dir}/scripts/build.sh first." >&2
        return 2
    fi
    source "${workspace_dir}/install/setup.bash"
}
