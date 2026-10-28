#!/usr/bin/env python3
"""
Trajectory Slew-Rate Limiter Node for G.L.A.D.

Applies independent acceleration and deceleration bounds to linear and angular
velocity commands, ensuring smooth motor transitions and preventing wheel slip.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TrajectoryController(Node):
    def __init__(self):
        super().__init__("trajectory_controller")

        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("accel_linear", 0.2)
        self.declare_parameter("decel_linear", 0.4)
        self.declare_parameter("accel_angular", 0.4)
        self.declare_parameter("decel_angular", 0.8)

        self.timeout = float(self.get_parameter("timeout").value)
        self.accel_linear = float(self.get_parameter("accel_linear").value)
        self.decel_linear = float(self.get_parameter("decel_linear").value)
        self.accel_angular = float(self.get_parameter("accel_angular").value)
        self.decel_angular = float(self.get_parameter("decel_angular").value)

        self.target = Twist()
        self.target_stamp = self.get_clock().now()
        self.current = Twist()

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel/raw", self.raw_cb, 10)

        self.create_timer(0.05, self.update)

        self.get_logger().info(
            f"Trajectory controller started — "
            f"accel: {self.accel_linear} m/s² / {self.accel_angular} rad/s², "
            f"decel: {self.decel_linear} m/s² / {self.decel_angular} rad/s²"
        )

    def raw_cb(self, msg: Twist):
        self.target = msg
        self.target_stamp = self.get_clock().now()

    @staticmethod
    def ramp(current: float, target: float, max_change: float) -> float:
        """Clamp change rate to [-max_change, +max_change] to prevent sudden jumps."""
        diff = target - current
        if abs(diff) < 1e-5:
            return target
        clamped_diff = max(-max_change, min(max_change, diff))
        return current + clamped_diff

    def update(self):
        dt = 0.05
        age = (self.get_clock().now() - self.target_stamp).nanoseconds / 1e9

        cmd = Twist()

        if age < self.timeout:
            target_x = self.target.linear.x
            target_z = self.target.angular.z
        else:
            target_x = 0.0
            target_z = 0.0

        # Choose decel rate if approaching zero, otherwise accel rate
        if abs(target_x) < abs(self.current.linear.x):
            max_x = self.decel_linear * dt
        else:
            max_x = self.accel_linear * dt

        if abs(target_z) < abs(self.current.angular.z):
            max_z = self.decel_angular * dt
        else:
            max_z = self.accel_angular * dt

        cmd.linear.x = self.ramp(self.current.linear.x, target_x, max_x)
        cmd.angular.z = self.ramp(self.current.angular.z, target_z, max_z)

        self.current = cmd
        if rclpy.ok():
            try:
                self.pub.publish(cmd)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
