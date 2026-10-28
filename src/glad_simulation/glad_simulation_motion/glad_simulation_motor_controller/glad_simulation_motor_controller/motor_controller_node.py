#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class MotorControllerBridge(Node):
    def __init__(self):
        super().__init__("motor_controller_bridge")
        self.pub = self.create_publisher(Odometry, "/motor/odom", 10)
        self.create_subscription(Odometry, "/odom/wheel", self.odom_cb, 10)
        self.get_logger().info(
            "Motor controller bridge started (simulation) — "
            "relaying /odom/wheel → /motor/odom"
        )

    def odom_cb(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerBridge()
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

