# RS motor SDK

`rs_motor_sdk` provides the SocketCAN transport and RobStride extended-CAN
protocol used by the reBotArm RS configuration. It supports RS-00 and RS-06
motors, MIT commands, enable/disable, software zero, parameter access, and
feedback decoding.

Read one six-axis feedback set without enabling the motors:

```bash
ros2 run rs_motor_sdk rs_motor_read_state can0
```

Continuously refresh the state table every 100 ms until `Ctrl+C`:

```bash
ros2 run rs_motor_sdk rs_motor_read_state can0 --watch 100
```

也可以直接运行工作区脚本：

```bash
scripts/motor/RS/motor_rs_read.sh
scripts/motor/RS/motor_rs_read.sh --refresh-ms 200
scripts/motor/RS/motor_rs_read.sh --once
```

All scripts under `scripts/motor/RS/` check the selected SocketCAN interface
before accessing hardware. An interface that is already active is left
unchanged. A down, stopped, or bus-off interface is automatically configured
for classic CAN at 1 Mbps with 100 ms bus-off restart and brought up using
`sudo`. Override these defaults with `RS_CAN_BITRATE` and
`RS_CAN_RESTART_MS` when required. `--dry-run` never changes the interface.

Set the current position as zero for one, multiple, or all arm joints. This is
a persistent calibration operation and requires both `--execute` and an
interactive `RS_ZERO` confirmation:

```bash
scripts/motor/RS/motor_rs_zero.sh --joint joint3 --execute
scripts/motor/RS/motor_rs_zero.sh --joints joint1,joint3,joint6 --execute
scripts/motor/RS/motor_rs_zero.sh --all --execute
```

Move one, multiple, or all joints to their calibrated zero position with a
minimum-jerk trajectory. The default duration is 5 seconds; use `--duration`
to select a duration from 1 to 120 seconds. The command enables and moves real
motors and requires an interactive `RS_HOME` confirmation.
Selected motors are disabled on exit:

Before motion, the tool preloads a torque-free command at each measured
position, then enables the selected motors one at a time and ramps each
motor's conservative homing gains for 1 second. This prevents an enable event
from briefly applying a stale target or full control gains and spreads
multi-axis startup load over time. These gains are intentionally lower than
the normal controller gains because high derivative gain amplifies standstill
velocity noise.

```bash
scripts/motor/RS/motor_rs_home.sh --joint joint3
scripts/motor/RS/motor_rs_home.sh --joints joint1,joint3,joint6 --duration 3
scripts/motor/RS/motor_rs_home.sh --all --dry-run
```

For hardware diagnosis, add `--verbose`. The tool prints the initial state,
each enable event, early control feedback, and detailed target/actual values
for any fault, including whether it came from command feedback or a periodic
active-report frame:

```bash
scripts/motor/RS/motor_rs_home.sh --all --duration 5 --verbose
```

Disable selected motors and clear their active fault state, then verify that
feedback reports fault code zero:

```bash
scripts/motor/RS/motor_rs_clear_fault.sh --joint joint3 --execute
scripts/motor/RS/motor_rs_clear_fault.sh --joints joint1,joint3,joint6 --execute
scripts/motor/RS/motor_rs_clear_fault.sh --all --execute
```

The hardware plugin supplies the robot-specific mapping. For `model=rs`, the
default configuration is motor IDs `0x01` through `0x06`, host ID `0xFD`,
RS-06 on joints 1--3, and RS-00 on joints 4--6.

Before activating real hardware, configure the connected SocketCAN adapter at
the motor bus bitrate and confirm the physical emergency stop is available.
Keep `allow_motor_enable:=false` until motor direction, zero offsets, and
joint limits have been validated.
