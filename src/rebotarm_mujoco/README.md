# rebotarm_mujoco

reBotArm 的 MuJoCo、ros2_control 与 MoveIt 2 集成包，同时支持 RS 和 DM。

## 启动

```bash
# MuJoCo GUI，默认 position 控制
ros2 launch rebotarm_mujoco simulation.launch.py model:=rs world:=empty

# 无界面 effort/PID；参数未经实机标定
ros2 launch rebotarm_mujoco headless_sim.launch.py model:=dm control_mode:=effort

# MuJoCo、MoveIt 2 和 RViz
ros2 launch rebotarm_mujoco moveit_sim.launch.py model:=rs world:=workcell
```

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model` | `rs` | `rs` 或 `dm` |
| `world` | `empty` | `empty` 或带桌面及方块的 `workcell` |
| `control_mode` | `position` | `position` 或未标定的 `effort` |
| `gui` | `true` | 是否显示 MuJoCo 窗口 |
| `enable_camera` | `false` | 发布腕部 RGB-D 数据；headless批处理保持关闭 |
| `sim_speed_factor` | `1.0` | MuJoCo 相对实时速度 |
| `random_seed` | `0` | 确定性运行契约；当前场景无随机采样 |

首次运行官方 URDF→MJCF 转换器时，会在 ROS home 下建立独立 Python 虚拟环境。

## 仿真接口

- 六轴轨迹：`/arm_controller/follow_joint_trajectory`
- 夹爪：`/gripper_controller/gripper_cmd`
- 仿真时钟：`/clock`
- 重置：`/mujoco_ros2_control_node/reset_world`
- 暂停：`/mujoco_ros2_control_node/set_pause`
- 单步：`/mujoco_ros2_control_node/step_simulation`
- 腕部六维力/力矩：ros2_control 传感器 `wrist_fts`
- 可选相机：`/wrist_camera/color/image_raw`、camera info 和对齐深度图

安装完整控制器运行依赖：

```bash
sudo apt install ros-humble-gripper-controllers ros-humble-force-torque-sensor-broadcaster
```

## MoveIt 2 示例

启动 `moveit_sim.launch.py` 后，可在 RViz 中以 `gripper_tcp` 为末端规划。只进行规划检查：

```bash
ros2 launch rebotarm_moveit_config planning_demo.launch.py model:=rs execute:=false
```

`execute:=true` 会依次执行回零、TCP位姿、直线接近、夹爪闭合、抬升和释放；执行前应先
确认 workcell、TCP 和碰撞代理符合当前机器人。

## 精度与扩展边界

`physical_parameters_rs.yaml` 和 `physical_parameters_dm.yaml` 中所有动力学、执行器和TCP
参数均明确标记为 `uncalibrated`，只能用于仿真开发，不能据此判断真机安全范围。

`domain_randomization.yaml` 预留强化学习随机化契约，默认关闭。`pinocchio_controller_interface.yaml`
定义未来模型控制器的 q/dq/ddq 输入和 effort 输出边界；当前不引入 Pinocchio，也不实现
重力补偿、阻抗控制或 MPC。
