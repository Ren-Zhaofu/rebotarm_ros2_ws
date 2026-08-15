import importlib.util
from pathlib import Path
import subprocess

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
JOINTS = [f"joint{index}" for index in range(1, 7)]


def load_launch(name):
    launch_file = PACKAGE_ROOT / "launch" / name
    specification = importlib.util.spec_from_file_location(name, launch_file)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module.generate_launch_description()


def launch_defaults(description):
    return {
        action.name: action.default_value[0].text
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument) and action.default_value
    }


def test_controller_contract():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "controllers.yaml").read_text(encoding="utf-8")
    )
    assert config["controller_manager"]["ros__parameters"]["update_rate"] == 100
    parameters = config["arm_controller"]["ros__parameters"]
    assert parameters["joints"] == JOINTS
    assert parameters["command_interfaces"] == ["position"]
    assert parameters["state_interfaces"] == ["position", "velocity"]


def test_controlled_xacro_expands_for_dm_and_rs_mock():
    xacro_file = PACKAGE_ROOT / "urdf" / "rebotarm_controlled.urdf.xacro"
    for model in ("dm", "rs"):
        result = subprocess.run(
            ["xacro", str(xacro_file), f"model:={model}"],
            check=True,
            capture_output=True,
            text=True,
        )
        assert '<ros2_control name="RebotArmSystem" type="system">' in result.stdout
        assert "<param name=\"transport\">mock</param>" in result.stdout
        for joint in JOINTS:
            assert f'<joint name="{joint}">' in result.stdout


def test_rs_socketcan_xacro_passes_safety_contract_without_current_writes():
    xacro_file = PACKAGE_ROOT / "urdf" / "rebotarm_controlled.urdf.xacro"
    result = subprocess.run(
        [
            "xacro",
            str(xacro_file),
            "model:=rs",
            "transport:=socketcan",
            "can_interface:=vcan0",
            "allow_motor_enable:=true",
            "hold_only:=true",
            "enable_on_controller_start:=true",
            "motor_enable_mask:=0,0,0,0,0,1",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    output = result.stdout
    assert '<param name="can_interface">vcan0</param>' in output
    assert '<param name="allow_motor_enable">True</param>' in output
    assert '<param name="hold_only">True</param>' in output
    assert '<param name="enable_on_controller_start">True</param>' in output
    assert '<param name="motor_enable_mask">0,0,0,0,0,1</param>' in output
    assert '<param name="mit_kp">10,15,15,15,12,10</param>' in output
    assert '<param name="mit_kd">0.3,0.3,0.3,0.4,0.3,0.3</param>' in output
    assert '<param name="rs_soft_start_ms">1000</param>' in output
    assert '<param name="max_tracking_error">0.2</param>' in output
    assert "7018" not in output
    assert "current_limit" not in output


def test_launch_defaults_to_rs_mock_and_read_only_hardware():
    control_defaults = launch_defaults(load_launch("rebotarm_control.launch.py"))
    assert control_defaults == {
        "model": "rs",
        "transport": "mock",
        "can_interface": "can0",
        "allow_motor_enable": "false",
        "hold_only": "true",
        "enable_on_controller_start": "false",
        "motor_enable_mask": "0,0,0,0,0,0",
        "start_arm_controller": "true",
        "arm_controller_start_stopped": "false",
    }
    hardware_defaults = launch_defaults(load_launch("rebotarm_hardware.launch.py"))
    assert hardware_defaults["model"] == "rs"
    assert hardware_defaults["allow_motor_enable"] == "false"
    assert hardware_defaults["hold_only"] == "true"


def test_arm_controller_condition_enforces_real_hardware_gates():
    description = load_launch("rebotarm_control.launch.py")
    spawners = [
        action
        for action in description.entities
        if isinstance(action, Node) and action.node_executable == "spawner"
    ]
    assert len(spawners) == 3
    assert spawners[0].condition is None
    active_condition = spawners[1].condition
    stopped_condition = spawners[2].condition

    cases = [
        ({"transport": "mock", "start_arm_controller": "true"}, True),
        ({"transport": "mock", "start_arm_controller": "false"}, False),
        (
            {
                "transport": "socketcan",
                "allow_motor_enable": "false",
                "hold_only": "true",
                "start_arm_controller": "true",
            },
            False,
        ),
        (
            {
                "transport": "socketcan",
                "allow_motor_enable": "true",
                "hold_only": "true",
                "start_arm_controller": "true",
            },
            False,
        ),
        (
            {
                "transport": "socketcan",
                "allow_motor_enable": "true",
                "hold_only": "false",
                "start_arm_controller": "true",
            },
            True,
        ),
    ]
    for configurations, expected in cases:
        context = LaunchContext()
        context.launch_configurations.update(
            {
                "allow_motor_enable": "false",
                "hold_only": "true",
                "arm_controller_start_stopped": "false",
                **configurations,
            }
        )
        assert active_condition.evaluate(context) is expected
        context.launch_configurations["arm_controller_start_stopped"] = "true"
        assert active_condition.evaluate(context) is False
        assert stopped_condition.evaluate(context) is expected
