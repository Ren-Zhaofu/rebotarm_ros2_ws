#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rebotarm_digital_twin.twin_utils import JOINT_NAMES, clamp_vector, rate_limit_vector


class TargetLimiter(Node):
    def __init__(self):
        super().__init__("target_limiter")
        self.declare_parameter("max_step_rad", 0.02)
        self.declare_parameter("update_rate_hz", 10.0)
        self.max_step = float(self.get_parameter("max_step_rad").value)
        update_rate = float(self.get_parameter("update_rate_hz").value)
        if update_rate <= 0.0:
            raise ValueError("update_rate_hz must be positive")
        self.current = (0.0,) * 6
        self.target = self.current
        self.initialized = False
        self.publisher = self.create_publisher(JointState, "/host/target_joint_states", 10)
        self.create_subscription(JointState, "/host/raw_joint_states", self.on_target, 10)
        self.create_timer(1.0 / update_rate, self.advance)

    def on_target(self, message):
        if tuple(message.name) != JOINT_NAMES or len(message.position) != 6:
            self.get_logger().warning("Ignoring host target with invalid joint contract")
            return
        self.target = clamp_vector(message.position)
        if not self.initialized:
            self.current = self.target
            self.initialized = True
            self.publish_current()

    def advance(self):
        if not self.initialized or self.current == self.target:
            return
        self.current = rate_limit_vector(self.current, self.target, self.max_step)
        self.publish_current()

    def publish_current(self):
        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = list(JOINT_NAMES)
        output.position = list(self.current)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = TargetLimiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
