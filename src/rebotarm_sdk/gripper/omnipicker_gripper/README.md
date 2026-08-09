# rebotarm_gripper_sdk

面向 reBotArm 工程的 Omnipicker 夹爪 C++17 / SocketCAN SDK。该包已移除厂商 USB2CANFD、设备 SN、预编译静态库和 Python 示例依赖，与机械臂电机 SDK 共用系统预配置的 SocketCAN 接口。

## 默认配置

- CAN 接口：命令行工具默认 `can0`
- 夹爪命令 CAN ID：`0x07`
- 夹爪反馈 CAN ID：`0x07`
- 帧类型：经典 CAN、11 位标准帧、8 字节
- 总线波特率：由系统负责配置，SDK 不修改网络接口

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

位置、速度、力度、加速度和减速度的有效范围均为 `[0.0, 1.0]`。命令工具连续发送 5 帧、间隔 10 ms，以兼容需要短时周期下发的固件；工具退出时不会自动发送归零命令，夹爪会保持目标。

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
