from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("rebotarm_description"))
    xacro_file = package_share / "urdf" / "rebotarm.urdf.xacro"

    model = LaunchConfiguration("model")
    joint_state_source = LaunchConfiguration("joint_state_source")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command(["xacro ", str(xacro_file), " model:=", model]),
        value_type=str,
    )
    gui_condition = IfCondition(
        PythonExpression(["'", joint_state_source, "' == 'gui'"])
    )
    headless_condition = IfCondition(
        PythonExpression(["'", joint_state_source, "' == 'headless'"])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="dm",
                choices=["dm", "rs"],
                description="reBotArm mechanical model",
            ),
            DeclareLaunchArgument(
                "joint_state_source",
                default_value="gui",
                choices=["gui", "headless", "none"],
                description="Joint state source: GUI, headless publisher, or external",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(package_share / "rviz" / "display.rviz"),
                description="RViz configuration file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "false"],
                description="Use the simulation clock",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=gui_condition,
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=headless_condition,
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
