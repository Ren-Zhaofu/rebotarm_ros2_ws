#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=motor_common.sh
source "${SCRIPT_DIR}/motor_common.sh"

usage() {
    echo "Usage: sudo $0 [--interface name] [slave-id|all] [print-period-ms]"
    echo "Example: sudo $0 all 100"
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
    usage
    exit 0
fi

rs_resolve_interface "$@"
set -- "${RS_MOTOR_ARGS[@]}"

if [[ $# -gt 2 ]]; then
    usage >&2
    exit 2
fi

rs_require_root
rs_require_sdk

python3 - "${EYOU_SDK_LIB}" "$RS_MOTOR_INTERFACE" "$@" <<'PY'
import ctypes
import os
import signal
import sys
import time

ETH_SUCCESS = 0

lib_dir, interface = sys.argv[1:3]
slave_arg = sys.argv[3] if len(sys.argv) >= 4 else "all"

try:
    print_period_ms = int(sys.argv[4]) if len(sys.argv) >= 5 else 100
    if not 1 <= print_period_ms <= 60000:
        raise ValueError
except ValueError:
    sys.exit("print-period-ms must be an integer from 1 to 60000")

if slave_arg == "all":
    requested_slave = None
else:
    try:
        requested_slave = int(slave_arg)
        if not 1 <= requested_slave <= 65535:
            raise ValueError
    except ValueError:
        sys.exit("slave-id must be 'all' or an integer from 1 to 65535")

try:
    ctypes.CDLL(os.path.join(lib_dir, "libsoem.so"), mode=ctypes.RTLD_GLOBAL)
    ctypes.CDLL(
        os.path.join(lib_dir, "libcyhcs_log.so"), mode=ctypes.RTLD_GLOBAL
    )
    sdk = ctypes.CDLL(
        os.path.join(lib_dir, "libeu_ethercat.so"), mode=ctypes.RTLD_GLOBAL
    )
except OSError as error:
    sys.exit("Failed to load EYou SDK: {}".format(error))

sdk.eth_initDLL.argtypes = [
    ctypes.c_char_p,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int),
]
sdk.eth_initDLL.restype = ctypes.c_int
sdk.eth_getActualPosition.argtypes = [
    ctypes.c_uint16,
    ctypes.POINTER(ctypes.c_int32),
]
sdk.eth_getActualPosition.restype = ctypes.c_int
sdk.eth_getActualVelocity.argtypes = [
    ctypes.c_uint16,
    ctypes.POINTER(ctypes.c_int32),
]
sdk.eth_getActualVelocity.restype = ctypes.c_int
sdk.eth_getActualTorque.argtypes = [
    ctypes.c_uint16,
    ctypes.POINTER(ctypes.c_int16),
]
sdk.eth_getActualTorque.restype = ctypes.c_int
sdk.eth_getTorqueSensorValue.argtypes = [
    ctypes.c_uint16,
    ctypes.POINTER(ctypes.c_int32),
]
sdk.eth_getTorqueSensorValue.restype = ctypes.c_int
sdk.eth_getStatusWord.argtypes = [
    ctypes.c_uint16,
    ctypes.POINTER(ctypes.c_uint16),
]
sdk.eth_getStatusWord.restype = ctypes.c_int
sdk.eth_getErrorCode.argtypes = [
    ctypes.c_uint16,
    ctypes.POINTER(ctypes.c_uint16),
]
sdk.eth_getErrorCode.restype = ctypes.c_int
sdk.eth_readSDO.argtypes = [
    ctypes.c_uint16,
    ctypes.c_uint16,
    ctypes.c_uint8,
    ctypes.c_void_p,
    ctypes.c_int,
    ctypes.c_int,
]
sdk.eth_readSDO.restype = ctypes.c_int
sdk.eth_freeDLL.argtypes = []
sdk.eth_freeDLL.restype = ctypes.c_int

stopping = False


def stop(_signum, _frame):
    global stopping
    stopping = True


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

slave_count = ctypes.c_int()
result = sdk.eth_initDLL(interface.encode(), 1, ctypes.byref(slave_count))
if result != ETH_SUCCESS:
    sys.exit(
        "EtherCAT initialization failed (error {}). "
        "Check the interface, cable, and permissions.".format(result)
    )

try:
    if requested_slave is not None and requested_slave > slave_count.value:
        sys.exit(
            "Slave {} requested, but only {} slave(s) found.".format(
                requested_slave, slave_count.value
            )
        )

    slaves = list(
        [requested_slave]
        if requested_slave is not None
        else range(1, slave_count.value + 1)
    )
    interactive = sys.stdout.isatty()
    temperatures = {}

    # eth_DataType_int32 is 0x04 in eu_ethercat.h.
    for slave in slaves:
        temperature = ctypes.c_int32()
        result = sdk.eth_readSDO(
            slave, 0x277A, 0x00, ctypes.byref(temperature), 0x04, 20000
        )
        if result == ETH_SUCCESS:
            temperatures[slave] = str(temperature.value)
        else:
            temperatures[slave] = "ERR({})".format(result)

    if interactive:
        # Use the alternate screen and hide the cursor while refreshing.
        sys.stdout.write("\033[?1049h\033[?25l\033[2J")
        sys.stdout.flush()

    columns = [
        ("Slave", 6),
        ("Temp(C)", 8),
        ("Position", 14),
        ("Velocity", 14),
        ("Torque", 10),
        ("Torque sensor", 14),
        ("Status", 8),
        ("Error", 8),
    ]
    header = "  ".join(
        "{:<{width}}".format(name, width=width)
        for name, width in columns
    )
    separator = "  ".join("-" * width for _name, width in columns)

    while not stopping:
        rows = []
        for slave in slaves:
            position = ctypes.c_int32()
            velocity = ctypes.c_int32()
            torque = ctypes.c_int16()
            torque_sensor = ctypes.c_int32()
            status_word = ctypes.c_uint16()
            error_code = ctypes.c_uint16()

            reads = [
                ("position", sdk.eth_getActualPosition, position),
                ("velocity", sdk.eth_getActualVelocity, velocity),
                ("torque", sdk.eth_getActualTorque, torque),
                (
                    "torque_sensor",
                    sdk.eth_getTorqueSensorValue,
                    torque_sensor,
                ),
                ("status_word", sdk.eth_getStatusWord, status_word),
                ("error_code", sdk.eth_getErrorCode, error_code),
            ]
            values = {}
            failed = []
            for name, reader, value in reads:
                result = reader(slave, ctypes.byref(value))
                if result == ETH_SUCCESS:
                    values[name] = value.value
                else:
                    failed.append("{}:{}".format(name, result))

            if failed:
                rows.append(
                    "{:<6}  {:>8}  {}".format(
                        slave,
                        temperatures[slave],
                        "read failed: " + ", ".join(failed),
                    )
                )
            else:
                rows.append(
                    "  ".join(
                        [
                            "{:<6}".format(slave),
                            "{:>8}".format(temperatures[slave]),
                            "{:>14}".format(values["position"]),
                            "{:>14}".format(values["velocity"]),
                            "{:>10}".format(values["torque"]),
                            "{:>14}".format(values["torque_sensor"]),
                            "{:>8}".format(
                                "0x{:04X}".format(values["status_word"])
                            ),
                            "{:>8}".format(
                                "0x{:04X}".format(values["error_code"])
                            ),
                        ]
                    )
                )

        if interactive:
            frame = [
                "EYou EtherCAT motor status",
                "Interface: {}   Detected: {}   Refresh: {} ms   "
                "Ctrl+C: quit".format(
                    interface, slave_count.value, print_period_ms
                ),
                "",
                header,
                separator,
            ]
            frame.extend(rows)
            sys.stdout.write(
                "\033[H" + "\n".join("\033[2K" + line for line in frame)
            )
            sys.stdout.write("\033[J")
            sys.stdout.flush()
        else:
            for row in rows:
                print(row, flush=True)
        time.sleep(print_period_ms / 1000.0)
finally:
    if "interactive" in locals() and interactive:
        sys.stdout.write("\033[?25h\033[?1049l")
        sys.stdout.flush()
    sdk.eth_freeDLL()
PY
