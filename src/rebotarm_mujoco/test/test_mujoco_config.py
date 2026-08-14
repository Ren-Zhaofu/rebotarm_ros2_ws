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
    assert [actuator.attrib["joint"] for actuator in actuators] == [f"joint{i}" for i in range(1, 7)]
