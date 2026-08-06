# RS motor SDK

`rs_motor_sdk` provides the SocketCAN transport and RobStride extended-CAN
protocol used by the reBotArm RS configuration. It supports RS-00 and RS-06
motors, MIT commands, enable/disable, software zero, parameter access, and
feedback decoding.

The hardware plugin supplies the robot-specific mapping. For `model=rs`, the
default configuration is motor IDs `0x01` through `0x06`, host ID `0xFD`,
RS-06 on joints 1--3, and RS-00 on joints 4--6.

Before activating real hardware, configure the connected SocketCAN adapter at
the motor bus bitrate and confirm the physical emergency stop is available.
Keep `allow_motor_enable:=false` until motor direction, zero offsets, and
joint limits have been validated.
