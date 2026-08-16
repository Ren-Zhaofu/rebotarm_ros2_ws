"""SocketCAN readiness checks used before starting real-feedback launches."""

import re
import subprocess


def check_socketcan_interface(interface):
    """Return whether a SocketCAN interface is usable and a user-facing reason."""
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", "dev", interface],
            check=False,
            capture_output=True,
            text=True,
            timeout=2,
        )
    except FileNotFoundError:
        return False, "The 'ip' command is unavailable; install iproute2 first."
    except subprocess.TimeoutExpired:
        return False, f"Timed out while checking SocketCAN interface '{interface}'."

    if result.returncode != 0:
        return False, (
            f"SocketCAN interface '{interface}' does not exist. "
            "Check the CAN adapter and its driver."
        )

    details = result.stdout
    if "link/can" not in details:
        return False, f"Network interface '{interface}' is not a SocketCAN interface."

    flags = re.search(r"<([^>]*)>", details)
    if flags is None or "UP" not in flags.group(1).split(","):
        return False, (
            f"SocketCAN '{interface}' is down. Initialize it with "
            f"./scripts/motor/dm/motor_dm_read.sh --interface {interface} --once"
        )

    state = re.search(r"can state ([A-Z-]+)", details)
    if state is None:
        return False, f"SocketCAN '{interface}' did not report a CAN state."
    if state.group(1) in {"STOPPED", "BUS-OFF"}:
        return False, (
            f"SocketCAN '{interface}' is {state.group(1)}. Initialize it with "
            f"./scripts/motor/dm/motor_dm_read.sh --interface {interface} --once"
        )

    return True, f"SocketCAN '{interface}' is UP ({state.group(1)})."
