"""Real-hardware bringup with read-only RS operation as the default."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("rebotarm_bringup"))
    model = LaunchConfiguration("model")
    can_interface = LaunchConfiguration("can_interface")
    allow_motor_enable = LaunchConfiguration("allow_motor_enable")
    hold_only = LaunchConfiguration("hold_only")
    enable_on_controller_start = LaunchConfiguration("enable_on_controller_start")
    motor_enable_mask = LaunchConfiguration("motor_enable_mask")
    start_arm_controller = LaunchConfiguration("start_arm_controller")
    arm_controller_start_stopped = LaunchConfiguration(
        "arm_controller_start_stopped"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="rs", choices=["rs", "dm"]),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "allow_motor_enable", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "hold_only", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument("enable_on_controller_start", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument(
                "motor_enable_mask", default_value="0,0,0,0,0,0"
            ),
            DeclareLaunchArgument(
                "start_arm_controller",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "arm_controller_start_stopped",
                default_value="false",
                choices=["true", "false"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup_share / "launch" / "rebotarm_control.launch.py")
                ),
                launch_arguments={
                    "model": model,
                    "transport": "socketcan",
                    "can_interface": can_interface,
                    "allow_motor_enable": allow_motor_enable,
                    "hold_only": hold_only,
                    "enable_on_controller_start": enable_on_controller_start,
                    "motor_enable_mask": motor_enable_mask,
                    "start_arm_controller": start_arm_controller,
                    "arm_controller_start_stopped": arm_controller_start_stopped,
                }.items(),
            ),
        ]
    )
