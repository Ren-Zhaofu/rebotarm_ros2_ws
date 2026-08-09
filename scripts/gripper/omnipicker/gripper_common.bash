#!/usr/bin/env bash

OMNIPICKER_DEFAULT_INTERFACE="${OMNIPICKER_CAN_INTERFACE:-can0}"
OMNIPICKER_DEFAULT_CAN_ID="${OMNIPICKER_CAN_ID:-0x07}"

validate_omnipicker_interface_name() {
  local interface=$1
  [[ $interface =~ ^[[:alnum:]_.-]+$ ]] || {
    echo "Invalid SocketCAN interface: $interface" >&2
    return 2
  }
}

validate_omnipicker_can_id() {
  local can_id=$1
  local numeric_id
  [[ $can_id =~ ^(0[xX][0-9a-fA-F]+|[0-9]+)$ ]] || {
    echo "Invalid CAN ID '$can_id'; expected an integer from 0 to 0x7FF." >&2
    return 2
  }
  numeric_id=$((can_id))
  (( numeric_id >= 0 && numeric_id <= 0x7FF )) || {
    echo "Invalid CAN ID '$can_id'; expected an integer from 0 to 0x7FF." >&2
    return 2
  }
}

validate_omnipicker_unit_value() {
  local label=$1
  local value=$2
  [[ $value =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] &&
    awk -v value="$value" 'BEGIN { exit !(value >= 0.0 && value <= 1.0) }' || {
      echo "$label must be a number from 0.0 to 1.0." >&2
      return 2
    }
}

ensure_omnipicker_can_interface() {
  local interface=$1
  local details

  command -v ip >/dev/null 2>&1 || {
    echo "The 'ip' command is required to inspect SocketCAN." >&2
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
  [[ $details == *"<"*"UP"*">"* ]] &&
    [[ $details != *"can state STOPPED"* ]] &&
    [[ $details != *"can state BUS-OFF"* ]] || {
      echo "SocketCAN '$interface' is not active; configure it before controlling the gripper." >&2
      return 1
    }
  echo "SocketCAN $interface is active; keeping its current configuration."
}

source_omnipicker_workspace() {
  local workspace_dir=$1
  set +u
  source /opt/ros/humble/setup.bash
  if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
    echo "Workspace is not built. Run ${workspace_dir}/scripts/build.sh first." >&2
    return 2
  fi
  source "${workspace_dir}/install/setup.bash"
  set -u
}
