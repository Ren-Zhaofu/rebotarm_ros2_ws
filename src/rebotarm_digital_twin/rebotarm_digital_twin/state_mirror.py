#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from rebotarm_digital_twin.twin_utils import JOINT_NAMES, within_limits


class StateMirror(Node):
    def __init__(self):
        super().__init__("state_mirror")
        self.declare_parameter("stale_timeout_sec", 0.1)
        self.declare_parameter("max_jump_rad", 0.5)
        self.timeout = float(self.get_parameter("stale_timeout_sec").value)
        self.max_jump = float(self.get_parameter("max_jump_rad").value)
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
        if tuple(message.name) != JOINT_NAMES or not within_limits(message.position):
            self.publish_status("FAULT", "invalid_joint_feedback")
            return
        positions = tuple(message.position)
        if self.last_positions is not None and any(
            abs(new - old) > self.max_jump
            for new, old in zip(positions, self.last_positions)
        ):
            self.publish_status("FAULT", "position_jump_exceeded")
            return
        mirrored = JointState()
        mirrored.header = message.header
        mirrored.name = list(JOINT_NAMES)
        mirrored.position = list(positions)
        if len(message.velocity) == 6:
            mirrored.velocity = list(message.velocity)
        if len(message.effort) == 6:
            mirrored.effort = list(message.effort)
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
