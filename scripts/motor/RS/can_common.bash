#!/usr/bin/env bash

ensure_rs_can_interface() {
  local interface=$1
  local bitrate=${RS_CAN_BITRATE:-1000000}
  local restart_ms=${RS_CAN_RESTART_MS:-100}
  local details
  local -a privilege=()

  [[ $bitrate =~ ^[0-9]+$ && $bitrate -gt 0 ]] || {
    echo "RS_CAN_BITRATE must be a positive integer." >&2
    return 2
  }
  [[ $restart_ms =~ ^[0-9]+$ ]] || {
    echo "RS_CAN_RESTART_MS must be a non-negative integer." >&2
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

  if [[ $details == *"<"*"UP"*">"* ]] &&
     [[ $details != *"can state STOPPED"* ]] &&
     [[ $details != *"can state BUS-OFF"* ]]; then
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
  "${privilege[@]}" ip link set dev "$interface" type can \
    bitrate "$bitrate" restart-ms "$restart_ms"
  "${privilege[@]}" ip link set dev "$interface" up

  details=$(ip -details link show dev "$interface")
  if [[ $details != *"<"*"UP"*">"* ]] ||
     [[ $details == *"can state STOPPED"* ]] ||
     [[ $details == *"can state BUS-OFF"* ]]; then
    echo "SocketCAN '$interface' did not enter an active state." >&2
    return 1
  fi
  echo "SocketCAN $interface is ready."
}
