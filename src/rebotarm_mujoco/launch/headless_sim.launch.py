from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = Path(get_package_share_directory("rebotarm_mujoco"))
    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    control_mode = LaunchConfiguration("control_mode")
    sim_speed_factor = LaunchConfiguration("sim_speed_factor")
    random_seed = LaunchConfiguration("random_seed")
    enable_ft_sensor = LaunchConfiguration("enable_ft_sensor")
    enable_camera = LaunchConfiguration("enable_camera")
    start_gripper_controller = LaunchConfiguration("start_gripper_controller")

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="rs", choices=["rs", "dm"]),
        DeclareLaunchArgument("world", default_value="empty", choices=["empty", "workcell"]),
        DeclareLaunchArgument("control_mode", default_value="position", choices=["position", "effort"]),
        DeclareLaunchArgument("sim_speed_factor", default_value="1.0"),
        DeclareLaunchArgument("random_seed", default_value="0"),
        DeclareLaunchArgument("enable_ft_sensor", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("enable_camera", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("start_gripper_controller", default_value="true", choices=["true", "false"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / "launch" / "simulation.launch.py")),
            launch_arguments={
                "model": model,
                "world": world,
                "control_mode": control_mode,
                "gui": "false",
                "enable_ft_sensor": enable_ft_sensor,
                "enable_camera": enable_camera,
                "start_gripper_controller": start_gripper_controller,
                "sim_speed_factor": sim_speed_factor,
                "random_seed": random_seed,
            }.items(),
        ),
    ])
