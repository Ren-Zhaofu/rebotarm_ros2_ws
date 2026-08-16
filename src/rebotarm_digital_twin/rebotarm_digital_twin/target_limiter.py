#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rebotarm_digital_twin.twin_utils import JOINT_NAMES, clamp_vector


def advance_motion(
    current,
    velocity,
    target,
    period,
    max_step,
    max_acceleration,
):
    if not all(len(values) == 6 for values in (current, velocity, target)):
        raise ValueError("joint vectors must contain exactly six values")
    if not all(
        math.isfinite(value) and value > 0.0
        for value in (period, max_step, max_acceleration)
    ):
        raise ValueError("motion limits must be finite and positive")

    max_velocity = max_step / period
    next_positions = []
    next_velocities = []
    for position, speed, destination in zip(current, velocity, target):
        error = destination - position
        direction = 1.0 if error > 0.0 else (-1.0 if error < 0.0 else 0.0)
        acceleration_step = max_acceleration * period
        if direction == 0.0 or speed * direction < 0.0:
            new_speed = math.copysign(
                max(0.0, abs(speed) - acceleration_step), speed
            )
        else:
            speed_magnitude = abs(speed)
            stopping_distance = speed_magnitude**2 / (2.0 * max_acceleration)
            if abs(error) <= stopping_distance + speed_magnitude * period:
                new_magnitude = max(0.0, speed_magnitude - acceleration_step)
            else:
                new_magnitude = min(
                    max_velocity, speed_magnitude + acceleration_step
                )
            new_speed = direction * new_magnitude
        new_position = position + 0.5 * (speed + new_speed) * period
        if (
            abs(destination - new_position) <= max_acceleration * period**2
            and abs(speed) <= acceleration_step
            and abs(new_speed) <= acceleration_step
        ):
            new_position = destination
            new_speed = 0.0
        next_positions.append(new_position)
        next_velocities.append(new_speed)
    return tuple(next_positions), tuple(next_velocities)


class TargetLimiter(Node):
    def __init__(self):
        super().__init__("target_limiter")
        self.declare_parameter("max_step_rad", 0.004)
        self.declare_parameter("max_acceleration_rad_s2", 0.6)
        self.declare_parameter("update_rate_hz", 50.0)
        self.max_step = float(self.get_parameter("max_step_rad").value)
        self.max_acceleration = float(
            self.get_parameter("max_acceleration_rad_s2").value
        )
        update_rate = float(self.get_parameter("update_rate_hz").value)
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (self.max_step, self.max_acceleration, update_rate)
        ):
            raise ValueError("motion parameters must be finite and positive")
        self.period = 1.0 / update_rate
        self.current = (0.0,) * 6
        self.velocity = (0.0,) * 6
        self.target = self.current
        self.initialized = False
        self.publisher = self.create_publisher(JointState, "/host/target_joint_states", 10)
        self.create_subscription(JointState, "/host/raw_joint_states", self.on_target, 10)
        self.create_timer(self.period, self.advance)

    def on_target(self, message):
        if tuple(message.name) != JOINT_NAMES or len(message.position) != 6:
            self.get_logger().warning("Ignoring host target with invalid joint contract")
            return
        self.target = clamp_vector(message.position)
        if not self.initialized:
            self.current = self.target
            self.velocity = (0.0,) * 6
            self.initialized = True
            self.publish_current()

    def advance(self):
        if not self.initialized or (
            self.current == self.target and self.velocity == (0.0,) * 6
        ):
            return
        self.current, self.velocity = advance_motion(
            self.current,
            self.velocity,
            self.target,
            self.period,
            self.max_step,
            self.max_acceleration,
        )
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
