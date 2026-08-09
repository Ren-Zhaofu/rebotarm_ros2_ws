# rebotarm_dm_motor_sdk

面向 reBotArm 的紧凑 C++17 达妙电机 SDK。它是一个标准 ament 包，只依赖 Linux
SocketCAN，不依赖 ROS 节点、`reference/` 源码树或其他工作空间。

## 内容

- `Protocol`：纯离线 CAN 帧编解码；
- `SocketCan`：经典 CAN 2.0B RAII 传输；
- `MotorBus`：电机注册、命令发送和反馈匹配；
- 状态命令：清错、使能、失能、保存位置零点；
- 控制命令：MIT、位置速度、速度、力位混控；
- 管理命令：刷新、读参数、写参数、保存参数；
- 状态、参数和保存应答解析。

SDK 不会在构造、连接或注册电机时自动发送任何 CAN 帧。`enable()`、`set_zero()`、
`write_parameter()` 和 `store_parameters()` 都必须由上层安全状态机显式调用。

当前 reBotArm DM 总线使用经典 CAN、1 Mbps：

```bash
sudo ip link set can0 down
sudo ip link set can0 mtu 16
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

SDK 只打开已经配置好的 SocketCAN 接口，不会自行执行提权命令或修改接口状态。

## reBotArm 真机实测

2026-07-26 在 `can0`、经典 CAN、1 Mbps 下对 1–7 号电机执行只读参数查询，
28/28 个请求均收到应答：

| 关节 | 命令/反馈 ID | CTRL_MODE | PMAX | VMAX | TMAX |
|---|---|---:|---:|---:|---:|
| joint1–joint3 | `0x01..0x03` / `0x11..0x13` | 2 | 12.5 | 10 | 28 |
| joint4–joint7 | `0x04..0x07` / `0x14..0x17` | 2 | 12.5 | 30 | 10 |

因此 joint1–joint3 的协议量程按 `kDm4340_48V` 处理，joint4–joint7 按
`kDm4310` 处理，不能将七个关节统一成同一量程。

安装后可运行只读诊断工具复核这些参数。该工具不包含使能、运动、写入或保存操作：

```bash
source install/setup.bash
ros2 run rebotarm_dm_motor_sdk dm_motor_read_parameters can0
```

可用状态读取工具单次查看或持续刷新机械臂六个关节的位置、速度、力矩、状态和温度。
刷新请求同样不会使能电机或写入参数：

```bash
ros2 run rebotarm_dm_motor_sdk dm_motor_read_state can0
ros2 run rebotarm_dm_motor_sdk dm_motor_read_state can0 --watch 100
```

在工作空间根目录也可以使用会自动检查并初始化 SocketCAN 的脚本：

```bash
./scripts/motor/dm/motor_dm_read.sh
./scripts/motor/dm/motor_dm_read.sh --once
```

将选中关节放到预期的 URDF 零位后，可用脚本把当前位置保存为电机软件零点。脚本会先
检查反馈和故障状态、失能电机并确认失能，然后写入零点并复核位置：

```bash
./scripts/motor/dm/motor_dm_zero.sh --joint joint3
./scripts/motor/dm/motor_dm_zero.sh --joints joint1,joint3,joint6
./scripts/motor/dm/motor_dm_zero.sh --all
```

该操作会永久改变电机软件零点，必须按提示输入 `DM_ZERO` 才会执行。可先添加
`--dry-run` 检查最终命令而不访问 CAN 总线。

## 六轴最小 jerk 回零

`dm_motor_minimum_jerk_home` 是需要显式 `--execute` 门禁的真机工具，仅控制机械臂
`joint1..joint6`，不控制尚未完成传动映射的 7 号夹爪电机：

```bash
source install/setup.bash
ros2 run rebotarm_dm_motor_sdk dm_motor_minimum_jerk_home --execute can0
```

工具在运动前只读复核六轴控制模式和 PMAX/VMAX/TMAX，要求六轴初始均为失能，
并按各轴机械范围分别检查起点（绝对值上限依次为
`2.80/3.14/3.14/1.87/1.57/3.14 rad`）。轨迹使用
`s(t)=10t³-15t⁴+6t⁵`，以 100 Hz 从各轴实时位置移动到电机坐标 `0 rad`，
时长至少 8 秒，模式 2 速度字段限制为 `0.05 rad/s`。状态异常、反馈超时、
跟踪误差超过 `0.08 rad`、发送失败或收到退出信号时都会发送两轮失能命令。
正常完成时要求最终误差不超过 `0.02 rad`，并只读确认六轴状态均恢复为失能。

2026-07-27 已完成首次六轴真机执行：起点为
`[-0.000191, 0.136378, -0.006676, 0.063897, 0.007057, 0.131800] rad`，
8 秒轨迹的峰值跟踪误差约 `0.0226 rad`；最终位置为
`[-0.000191, 0.000572, -0.000191, -0.000191, -0.000191, -0.000191] rad`，
六轴失能确认通过，CAN 总线错误计数保持为 0。

同日进一步用较大起点
`[-0.569734, 0.136378, -0.013161, 0.004005, 0.478943, 0.162318] rad`
验证按轴范围检查和自动时长计算：轨迹自动延长到 `26.706 s`，峰值跟踪误差约
`0.02994 rad`，最终六轴误差均小于 `0.001 rad`，失能确认和 CAN 错误检查通过。

这里的目标是当前电机编码器坐标零位，不等同于已经完成标定的 URDF/MoveIt 机械零位。
软件失能也不能替代可立即操作的硬件急停或动力断电。

## 最小使用方式

```cpp
#include "rebotarm_dm_motor_sdk/dm_motor.hpp"

using namespace rebotarm_dm_motor_sdk;

MotorBus bus;
bus.add_motor({"joint1", 0x01, 0x11, MotorModel::kDm4310});
if (!bus.connect("can0")) {
  throw std::runtime_error(bus.last_error());
}

// 只读查询，不会使能电机。
bus.request_state(0);
MotorState state;
std::size_t index = 0;
if (bus.receive_state(state, index, std::chrono::milliseconds(100))) {
  // 使用 state.position/state.velocity/state.effort。
}
```

任何使能、运动、设零或参数写入操作都应置于 `rebotarm_driver` 的安全门禁之后，不能在
SDK 示例中自动执行。

## 构建和测试

```bash
cd rebuild/rebotarm_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rebotarm_dm_motor_sdk
colcon test --packages-select rebotarm_dm_motor_sdk
colcon test-result --verbose
```

全部测试只检查协议字节，不打开 SocketCAN。
