"""SocketCAN readiness checks used before starting real-feedback launches."""

import re
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_prefix


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


def check_motor_feedback(interface, model):
    """Request one read-only state sample and require all configured motors."""
    tools = {
        "dm": ("rebotarm_dm_motor_sdk", "dm_motor_read_state"),
        "rs": ("rs_motor_sdk", "rs_motor_read_state"),
    }
    try:
        package, executable = tools[model]
    except KeyError:
        return False, f"Unsupported motor model '{model}' for feedback preflight."

    try:
        prefix = get_package_prefix(package)
    except PackageNotFoundError:
        return False, f"Motor feedback tool for model '{model}' is not installed."

    try:
        result = subprocess.run(
            [f"{prefix}/lib/{package}/{executable}", interface],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except subprocess.TimeoutExpired:
        return False, f"Timed out waiting for {model.upper()} motor feedback on '{interface}'."

    if result.returncode != 0:
        return False, (
            f"No complete {model.upper()} motor feedback on '{interface}'. "
            "Check motor power, CANH/CANL wiring, 120-ohm termination, "
            "the selected CAN channel, and the 1 Mbps bitrate."
        )

    return True, f"Complete {model.upper()} motor feedback received on '{interface}'."
