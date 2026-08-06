from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


SUPPORTED_MODELS = ("dm", "rs")


def _launch_setup(context):
    model = LaunchConfiguration("model").perform(context)
    if model not in SUPPORTED_MODELS:
        choices = ", ".join(SUPPORTED_MODELS)
        raise RuntimeError(f"Unsupported model '{model}'. Choose one of: {choices}")

    package_share = Path(get_package_share_directory("rebotarm_description"))
    xacro_file = package_share / "urdf" / model / "urdf" / f"rebotarm_{model}.urdf.xacro"
    robot_description = xacro.process_file(str(xacro_file)).toxml()
    rviz_config = package_share / "rviz" / "display.rviz"

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="dm",
                description="Robot model: dm or rs",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
