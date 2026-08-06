# reBotArm motor tools

本目录按机器人型号和电机总线分开维护真机诊断脚本：

- `DM/`：达妙电机，使用 Linux SocketCAN，默认接口为 `can0`。
- `RS/`：EYou 关节模组，使用 EtherCAT，默认自动检测独立有线网卡。

两套脚本的接口和协议不能混用。首次测试先运行对应目录下的
`motor_interface.sh`，确认物理接口，再运行只读命令。

## DM

```bash
scripts/motor/DM/motor_interface.sh
scripts/motor/DM/motor_read_parameters.sh
scripts/motor/DM/motor_home.sh --help
```

`motor_read_parameters.sh` 只读取电机参数，不使能电机。
`motor_home.sh` 会运动六个机械臂关节，必须显式提供 `--execute`。

## RS

先配置厂商 SDK 动态库目录：

```bash
export EYOU_SDK_LIB=/absolute/path/to/eyou_sdk/lib
```

然后检查接口和读取状态：

```bash
scripts/motor/RS/motor_interface.sh
sudo -E scripts/motor/RS/motor_read_raw.sh all 100
```

RS 第一阶段只提供接口诊断和只读状态读取，不提供使能、回零、制动器释放或运动命令。
确认实机型号、从站顺序、标定参数和急停条件后，再增加写入类工具。

## Safety

- 软件失能不能替代硬件急停和动力断电。
- 确认机器人周围无人、机械限位可靠并可立即断电后，才能执行运动命令。
- 不要同时运行多个 CAN 或 EtherCAT 主站访问同一物理接口。
- 厂商 SDK、固件、关节映射或标定参数不匹配时，不要尝试使能电机。
