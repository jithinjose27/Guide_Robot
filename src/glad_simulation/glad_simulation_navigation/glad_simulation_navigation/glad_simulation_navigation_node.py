#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class GladSimulationNavigationNode(Node):
    def __init__(self):
        super().__init__("glad_simulation_navigation_node")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel/nav", 10)

        self.get_logger().info("GladSimulationNavigationNode started")

    def destroy_node(self):
        self.get_logger().info("GladSimulationNavigationNode shutting down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GladSimulationNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
