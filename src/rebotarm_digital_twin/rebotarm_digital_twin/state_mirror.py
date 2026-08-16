#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from rebotarm_digital_twin.twin_utils import (
    JOINT_NAMES,
    joint_limits,
    ordered_joint_values,
    within_limits,
)

DISPLAY_GRIPPER_JOINTS = ("gripper_joint1", "gripper_joint2")
DISPLAY_JOINT_NAMES = JOINT_NAMES + DISPLAY_GRIPPER_JOINTS


def with_fixed_gripper(values):
    """Append the temporary closed gripper state required for RViz TF."""
    return tuple(float(value) for value in values) + (0.0, 0.0)


class StateMirror(Node):
    def __init__(self):
        super().__init__("state_mirror")
        self.declare_parameter("model", "dm")
        self.declare_parameter("stale_timeout_sec", 0.1)
        self.declare_parameter("max_jump_rad", 0.5)
        self.declare_parameter("feedback_limit_tolerance_rad", 0.005)
        self.timeout = float(self.get_parameter("stale_timeout_sec").value)
        self.max_jump = float(self.get_parameter("max_jump_rad").value)
        self.limit_tolerance = float(
            self.get_parameter("feedback_limit_tolerance_rad").value
        )
        self.model = str(self.get_parameter("model").value)
        joint_limits(self.model)
        self.started_at = time.monotonic()
        self.last_message_time = 0.0
        self.last_positions = None
        self.last_status = "STARTING"
        self.publisher = self.create_publisher(JointState, "/twin/joint_states", 10)
        self.status_publisher = self.create_publisher(String, "/twin/status", 10)
        self.create_subscription(
            JointState, "/real/joint_states", self.on_state, 10
        )
        self.timer = self.create_timer(0.02, self.on_timer)
        self.publish_status("FEEDBACK_ONLY")

    def publish_status(self, mode, reason=""):
        message = String()
        message.data = json.dumps(
            {"mode": mode, "reason": reason, "timestamp": time.time()},
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)
        self.last_status = mode

    def on_state(self, message):
        try:
            positions = ordered_joint_values(message.name, message.position)
        except ValueError:
            self.publish_status("FAULT", "invalid_joint_feedback")
            return
        if not within_limits(positions, self.model, self.limit_tolerance):
            self.publish_status("FAULT", "invalid_joint_feedback")
            return
        if self.last_positions is not None and any(
            abs(new - old) > self.max_jump
            for new, old in zip(positions, self.last_positions)
        ):
            self.publish_status("FAULT", "position_jump_exceeded")
            return
        mirrored = JointState()
        mirrored.header = message.header
        # The DM/RS arm feedback contains six axes only. Keep the unreconciled
        # gripper closed in the display until its hardware URDF is supplied.
        mirrored.name = list(DISPLAY_JOINT_NAMES)
        mirrored.position = list(with_fixed_gripper(positions))
        if len(message.velocity) == 6:
            mirrored.velocity = list(
                with_fixed_gripper(
                    ordered_joint_values(message.name, message.velocity)
                )
            )
        if len(message.effort) == 6:
            mirrored.effort = list(
                with_fixed_gripper(
                    ordered_joint_values(message.name, message.effort)
                )
            )
        self.publisher.publish(mirrored)
        self.last_positions = positions
        self.last_message_time = time.monotonic()
        if self.last_status != "FEEDBACK_ONLY":
            self.publish_status("FEEDBACK_ONLY")

    def on_timer(self):
        reference = self.last_message_time or self.started_at
        if (
            time.monotonic() - reference > self.timeout
            and self.last_status != "STALE"
        ):
            self.publish_status("STALE", "feedback_timeout")


def main(args=None):
    rclpy.init(args=args)
    node = StateMirror()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
