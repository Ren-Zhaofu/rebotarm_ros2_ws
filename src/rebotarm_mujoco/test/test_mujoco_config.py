from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).parents[1]


def test_worlds_are_valid_mjcf():
    for name in ("empty", "workcell"):
        root = ET.parse(PACKAGE / "mjcf" / "worlds" / f"{name}.xml").getroot()
        assert root.tag == "mujoco"


def test_position_controller_covers_six_arm_joints():
    config = yaml.safe_load((PACKAGE / "config" / "controllers_position.yaml").read_text())
    params = config["arm_controller"]["ros__parameters"]
    assert params["joints"] == [f"joint{i}" for i in range(1, 7)]
    assert params["command_interfaces"] == ["position"]


def test_position_actuators_cover_six_arm_joints():
    root = ET.parse(PACKAGE / "mjcf" / "position_actuators.xml").getroot()
    actuators = root.findall("./raw_inputs/actuator/position")
    assert [actuator.attrib["joint"] for actuator in actuators[:6]] == [f"joint{i}" for i in range(1, 7)]
    assert actuators[-1].attrib["joint"] == "gripper_joint1"


def test_actuator_variants_exclude_known_neutral_pose_self_collisions():
    expected = {
        ("world", "link1"),
        ("link2", "link4"),
        ("link4", "link6"),
        ("gripper_left", "gripper_right"),
    }
    for mode in ("position", "effort"):
        for suffix in ("", "_camera"):
            root = ET.parse(PACKAGE / "mjcf" / f"{mode}_actuators{suffix}.xml").getroot()
            excludes = root.findall("./raw_inputs/contact/exclude")
            assert {(item.attrib["body1"], item.attrib["body2"]) for item in excludes} == expected


def test_effort_controller_and_actuators_match():
    config = yaml.safe_load((PACKAGE / "config" / "controllers_effort.yaml").read_text())
    params = config["arm_controller"]["ros__parameters"]
    assert params["command_interfaces"] == ["effort"]
    assert set(params["gains"]) == {f"joint{i}" for i in range(1, 7)}
    root = ET.parse(PACKAGE / "mjcf" / "effort_actuators.xml").getroot()
    motors = root.findall("./raw_inputs/actuator/motor")
    assert [motor.attrib["joint"] for motor in motors] == [f"joint{i}" for i in range(1, 7)]


def test_physical_parameters_are_explicitly_uncalibrated():
    for model in ("rs", "dm"):
        config = yaml.safe_load((PACKAGE / "config" / f"physical_parameters_{model}.yaml").read_text())
        assert config["metadata"]["status"] == "uncalibrated"
        assert config["metadata"]["safety_use_allowed"] is False
        assert len(config["actuator_effort_limits"]) == 6


def test_extension_contracts_are_safe_by_default():
    randomization = yaml.safe_load((PACKAGE / "config" / "domain_randomization.yaml").read_text())
    assert randomization["enabled"] is False
    assert randomization["seed"] == 0
    pinocchio = yaml.safe_load((PACKAGE / "config" / "pinocchio_controller_interface.yaml").read_text())
    assert pinocchio["implemented"] is False
    assert pinocchio["command"]["interface"] == "effort"
    assert pinocchio["safety"]["reject_uncalibrated_real_hardware"] is True


def test_wrist_sensor_definitions_exist_in_both_control_modes():
    for mode in ("position", "effort"):
        root = ET.parse(PACKAGE / "mjcf" / f"{mode}_actuators.xml").getroot()
        names = {sensor.attrib["name"] for sensor in root.findall("./raw_inputs/sensor/*")}
        assert names == {"wrist_force", "wrist_torque"}
        assert root.find("./processed_inputs/camera") is None
        camera_root = ET.parse(PACKAGE / "mjcf" / f"{mode}_actuators_camera.xml").getroot()
        camera = camera_root.find("./processed_inputs/camera")
        assert camera.attrib["name"] == "wrist_camera"


def test_optional_gripper_controller_is_disabled_by_default():
    for launch_file in ("simulation.launch.py", "headless_sim.launch.py", "moveit_sim.launch.py"):
        source = (PACKAGE / "launch" / launch_file).read_text()
        declaration = source.split('DeclareLaunchArgument("start_gripper_controller"', 1)[1]
        assert 'default_value="false"' in declaration.split(")", 1)[0]


def test_gripper_dependency_failure_has_install_guidance():
    source = (PACKAGE / "launch" / "simulation.launch.py").read_text()
    assert 'get_package_prefix("gripper_controllers")' in source
    assert "sudo apt install ros-humble-gripper-controllers" in source
