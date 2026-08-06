# 机器人描述包协作规范

## 模块职责

- 本功能包只负责 reBotArm 的机器人几何描述、网格资源、TF 结构和 RViz 可视化。
- 同时维护 DM 和 RS 两种型号，修改公共行为时必须考虑两套模型。
- 不要在本功能包中实现硬件通信、控制器、运动规划或业务逻辑。

## 文件组织

- DM 模型资源放在 `urdf/dm/`，RS 模型资源放在 `urdf/rs/`。
- 各型号的 URDF/Xacro 放在对应型号的 `urdf/` 子目录。
- 网格文件放在对应型号的 `meshes/` 子目录。
- 通用启动文件放在 `launch/`，RViz 配置放在 `rviz/`。
- 资源引用使用 `package://rebotarm_description/...`，不要使用本机绝对路径。

## 模型修改要求

- 保持 link、joint 和 TF frame 命名稳定；需要重命名时说明兼容性影响。
- 保持 `base_footprint` 作为模型根坐标系，并保留 `gripper_tcp` 末端坐标系。
- 修改关节原点、轴向、限制或 TCP 后，检查完整父子层级和运动方向。
- 新增或替换网格时，同时检查 visual 和 collision 引用。
- CAD 导出的质量、惯量、关节限制、速度和力矩必须依据硬件规格复核后才能视为可信参数。
- 用于运动规划或仿真前，优先提供简化碰撞几何，不要默认使用高精度视觉网格。

## 必要验证

- 修改后至少编译本功能包：
  `./scripts/build.sh --packages-select rebotarm_description`
- DM 和 RS 的 Xacro 都必须能够成功展开和解析。
- 涉及启动或可视化时，分别检查：
  `ros2 launch rebotarm_description display.launch.py model:=dm`
  `ros2 launch rebotarm_description display.launch.py model:=rs`
- 如果图形环境不可用，说明未执行 RViz 人工检查，但仍需完成编译和模型解析检查。

## 修改边界

- 不要直接修改 `build/`、`install/` 或 `log/` 中的文件。
- 不要提交 Python 缓存或其他生成文件。
- 一次提交只处理一种模型问题或一种基础设施改动。
