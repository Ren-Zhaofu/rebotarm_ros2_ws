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

Set the current position as zero for one, multiple, or all arm joints. This is
a persistent calibration operation and requires both `--execute` and an
interactive `RS_ZERO` confirmation:

```bash
scripts/motor/RS/motor_rs_zero.sh --joint joint3 --execute
scripts/motor/RS/motor_rs_zero.sh --joints joint1,joint3,joint6 --execute
scripts/motor/RS/motor_rs_zero.sh --all --execute
```

Move one, multiple, or all joints gradually to their calibrated zero position.
The command enables and moves real motors, so it requires both `--execute` and
an interactive `RS_HOME` confirmation. Selected motors are disabled on exit:

```bash
scripts/motor/RS/motor_rs_home.sh --joint joint3 --execute
scripts/motor/RS/motor_rs_home.sh --joints joint1,joint3,joint6 --execute
scripts/motor/RS/motor_rs_home.sh --all --execute
```

The hardware plugin supplies the robot-specific mapping. For `model=rs`, the
default configuration is motor IDs `0x01` through `0x06`, host ID `0xFD`,
RS-06 on joints 1--3, and RS-00 on joints 4--6.

Before activating real hardware, configure the connected SocketCAN adapter at
the motor bus bitrate and confirm the physical emergency stop is available.
Keep `allow_motor_enable:=false` until motor direction, zero offsets, and
joint limits have been validated.
