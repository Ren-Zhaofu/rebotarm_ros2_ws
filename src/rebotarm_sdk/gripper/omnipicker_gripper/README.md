# rebotarm_gripper_sdk

面向 reBotArm 工程的 Omnipicker 夹爪 C++17 / SocketCAN SDK。该包已移除厂商 USB2CANFD、设备 SN、预编译静态库和 Python 示例依赖，与机械臂电机 SDK 共用系统预配置的 SocketCAN 接口。

## 默认配置

- CAN 接口：命令行工具默认 `can0`
- 夹爪命令 CAN ID：`0x07`
- 夹爪反馈 CAN ID：`0x07`
- 帧类型：经典 CAN、11 位标准帧、8 字节
- 总线波特率：脚本默认以 `1 Mbps` 自动上线未启动的接口
- Bus-Off 自动恢复：脚本默认配置 `restart-ms 100`

例如：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 编译

```bash
./scripts/build.sh --packages-select rebotarm_gripper_sdk
source install/setup.bash
```

## 命令行使用

日常调试优先使用工作空间脚本。它们会检查参数和工作空间；接口已经健康时保留当前配置，接口处于 DOWN、STOPPED 或 BUS-OFF 时自动按 `1 Mbps` 重新上线：

```bash
./scripts/gripper/omnipicker/gripper_open.sh
./scripts/gripper/omnipicker/gripper_close.sh --velocity 0.5 --force 0.6
./scripts/gripper/omnipicker/gripper_move.sh --position 0.5 --velocity 0.6 --force 0.7
./scripts/gripper/omnipicker/gripper_read.sh
./scripts/gripper/omnipicker/gripper_read.sh --once
```

所有脚本均支持 `--interface`、`--can-id` 和 `--dry-run`；默认使用 `can0` 和 `0x07`。移动脚本支持 `--duration-ms`，默认以 10 ms 周期下发 1000 ms，并要求收到无故障反馈后才报告成功。

可通过 `OMNIPICKER_CAN_INTERFACE`、`OMNIPICKER_CAN_ID`、`OMNIPICKER_CAN_BITRATE`、`OMNIPICKER_CAN_RESTART_MS` 和 `OMNIPICKER_COMMAND_DURATION_MS` 修改默认配置。若没有收到反馈，工具会返回失败；这通常需要检查夹爪供电、CAN_H/CAN_L、共地、终端电阻、波特率及反馈 ID。

底层 ROS 2 工具用法如下。

发送目标位置（默认速度和力度均为 1.0，默认 `can0` / `0x07`）：

```bash
ros2 run rebotarm_gripper_sdk gripper_command 1.0
ros2 run rebotarm_gripper_sdk gripper_command 0.5 0.6 0.7
ros2 run rebotarm_gripper_sdk gripper_command 0.0 1.0 1.0 can0 0x07
```

读取一帧反馈：

```bash
ros2 run rebotarm_gripper_sdk gripper_read_state
ros2 run rebotarm_gripper_sdk gripper_read_state can0 0x07 1000
```

位置、速度、力度、加速度和减速度的有效范围均为 `[0.0, 1.0]`。命令工具默认以 10 ms 周期持续发送 1000 ms，并解析反馈确认设备在线；工具退出时不会自动发送归零命令，夹爪会保持目标。

## C++ 使用

```cpp
#include "rebotarm_gripper_sdk/gripper.hpp"

rebotarm_gripper_sdk::Gripper gripper;
if (!gripper.connect("can0")) {
  // gripper.last_error()
}
gripper.send({0.5, 1.0, 0.7, 1.0, 1.0});
```

安全停机帧可通过 `gripper.stop()` 发送。它会将五个控制字段全部置零；业务层应根据夹爪和工件的实际安全要求决定何时调用。
