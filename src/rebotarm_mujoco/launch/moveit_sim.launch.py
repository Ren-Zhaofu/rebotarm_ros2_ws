from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mujoco = Path(get_package_share_directory("rebotarm_mujoco"))
    moveit = Path(get_package_share_directory("rebotarm_moveit_config"))
    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    control_mode = LaunchConfiguration("control_mode")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    start_gripper_controller = LaunchConfiguration("start_gripper_controller")

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="rs", choices=["rs", "dm"]),
        DeclareLaunchArgument("world", default_value="workcell", choices=["empty", "workcell"]),
        DeclareLaunchArgument("control_mode", default_value="position", choices=["position", "effort"]),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_gripper_controller", default_value="false", choices=["true", "false"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mujoco / "launch" / "simulation.launch.py")),
            launch_arguments={"model": model, "world": world, "control_mode": control_mode, "gui": gui,
                              "start_gripper_controller": start_gripper_controller}.items(),
        ),
        TimerAction(period=7.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit / "launch" / "move_group.launch.py")),
            launch_arguments={"model": model}.items(),
        )]),
        TimerAction(period=8.0, actions=[Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(moveit / "rviz" / "moveit.rviz")],
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(rviz),
        )]),
    ])
