# rebotarm_digital_twin

This package implements the safe digital-twin path for reBotArm. Real
feedback, host drag targets, simulation commands, and approved real commands
use separate topics. The default real mode is read-only.

## Read-only real feedback

Run `ros2 launch rebotarm_digital_twin feedback_twin.launch.py model:=dm`.
The launch defaults to `allow_motor_enable:=false`, an all-zero enable mask,
and no active arm trajectory controller. DM feedback is published on
`/real/joint_states`; the mirrored model state is `/twin/joint_states`. Twin
health is JSON on `/twin/status` and reports `FEEDBACK_ONLY`, `STALE`, or
`FAULT`.

## Host-only simulation drag

Run `ros2 launch rebotarm_digital_twin host_sim.launch.py model:=dm`. The GUI
publishes `/host/raw_joint_states`. The limiter applies the DM URDF limits and
minimum-jerk interpolation before publishing `/host/target_joint_states`. The
trajectory bridge commands MuJoCo through `/arm_controller/joint_trajectory`. This
launch uses `SIM_ONLY` and never opens SocketCAN.

## Real execution gate

The arbiter exposes `/real/arm` and `/real/set_execution`. Arming requires
fresh six-axis feedback. Execution first aligns the command target to the
latest feedback, then activates `arm_controller` through the namespaced
controller-manager service. Only that controller switch can trigger the
hardware plugin's deferred motor enable. Approved targets flow from
`/real/command_target` to `/real/arm_controller/joint_trajectory`. Explicit
disarm, invalid feedback, feedback timeout, or controller stop disables the
selected motors while retaining read-only feedback when possible.

The default launch remains read-only. Do not pass
`allow_motor_enable:=true` until single-axis validation, a working hardware
e-stop, and a tested manual power-off path are present.
