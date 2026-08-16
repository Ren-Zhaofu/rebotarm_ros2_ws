from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mujoco = Path(get_package_share_directory("rebotarm_mujoco"))
    model = LaunchConfiguration("model")
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="dm", choices=["dm", "rs"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mujoco / "launch" / "simulation.launch.py")),
            launch_arguments={
                "model": model,
                "world": "empty",
                "control_mode": "position",
                "gui": "true",
                "start_gripper_controller": "false",
            }.items(),
        ),
        Node(
            package="rebotarm_digital_twin",
            executable="joint_control_gui.py",
            parameters=[{"model": model}],
            output="screen",
        ),
        Node(
            package="rebotarm_digital_twin",
            executable="target_limiter.py",
            parameters=[{"model": model}],
            output="screen",
        ),
        Node(
            package="rebotarm_digital_twin",
            executable="target_to_trajectory.py",
            output="screen",
        ),
        Node(
            package="rebotarm_digital_twin",
            executable="mode_arbiter.py",
            parameters=[{"initial_mode": "SIM_ONLY", "model": model}],
            output="screen",
        ),
    ])
