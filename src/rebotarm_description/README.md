# rebotarm_description

reBotArm DM、RS 两种型号的机器人几何、TF、网格资源和 RViz 可视化功能包。

## 功能

- `urdf/rebotarm.urdf.xacro`：统一模型入口，通过 `model:=dm|rs` 选择型号
- `urdf/dm`、`urdf/rs`：各型号的 CAD URDF、Xacro 包装和 STL 网格
- `launch/display.launch.py`：机器人状态发布、关节状态输入和 RViz 启动
- `rviz/display.rviz`：包含 RobotModel、TF、基座坐标轴和末端 TCP 坐标轴

## 编译

```bash
./scripts/build.sh --packages-select rebotarm_description
source install/setup.bash
```

## RViz 可视化

使用关节滑块显示 DM 或 RS 模型：

```bash
ros2 launch rebotarm_description display.launch.py model:=dm
ros2 launch rebotarm_description display.launch.py model:=rs
```

无图形关节发布器适用于 CI、远程主机或只检查 TF 的场景：

```bash
ros2 launch rebotarm_description display.launch.py \
  model:=dm joint_state_source:=headless use_rviz:=false
```

使用 ros2_control、实机驱动或其他节点提供的 `/joint_states`：

```bash
ros2 launch rebotarm_description display.launch.py \
  model:=rs joint_state_source:=none
```

可用启动参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model` | `dm` | `dm` 或 `rs` |
| `joint_state_source` | `gui` | `gui`、`headless` 或 `none` |
| `use_rviz` | `true` | 是否启动 RViz |
| `rviz_config` | 包内配置 | 自定义 RViz 配置路径 |
| `use_sim_time` | `false` | 是否使用仿真时钟 |

统一 Xacro 也可以独立展开：

```bash
xacro $(ros2 pkg prefix rebotarm_description)/share/rebotarm_description/urdf/rebotarm.urdf.xacro model:=rs
```

## 验证

```bash
./scripts/build.sh --packages-select rebotarm_description
colcon test --packages-select rebotarm_description --event-handlers console_direct+
colcon test-result --verbose
```

测试会检查 DM、RS 的模型树、六个机械臂关节、夹爪 mimic 关系、网格路径、
Xacro/URDF 解析，以及关节状态输入能否产生对应的动态 TF。

## 模型数据状态

当前质量、惯量、关节限制、力矩和速度来自 CAD 导出。当前软件测试只能证明模型按
URDF 定义正常显示和运动，不能证明实机正方向、编码器零位、机械限位或 TCP 已完成
校准。用于动力学、安全限制或硬件控制前，必须依据机械和执行器规格复核这些参数。

当前 collision 复用了高精度视觉网格。进入 MoveIt 或仿真阶段前，应另外制作简化
碰撞几何，不应直接把当前网格视为最终规划模型。
