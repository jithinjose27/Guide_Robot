#!/usr/bin/env python3
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

        self.timeout = self.get_parameter("timeout").value
        self.accel_linear = self.get_parameter("accel_linear").value
        self.decel_linear = self.get_parameter("decel_linear").value
        self.accel_angular = self.get_parameter("accel_angular").value
        self.decel_angular = self.get_parameter("decel_angular").value

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

    def raw_cb(self, msg):
        self.target = msg
        self.target_stamp = self.get_clock().now()

    @staticmethod
    def ramp(current, target, max_change):
        diff = target - current
        if abs(diff) < 1e-6:
            return target
        return current + max(max(-max_change, diff), -max_change)

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
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
