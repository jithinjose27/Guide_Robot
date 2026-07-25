#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class GladEkfNode(Node):
    def __init__(self):
        super().__init__("glad_ekf_node")

        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("two_d_mode", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.get_logger().info("GladEkfNode started")

    def destroy_node(self):
        self.get_logger().info("GladEkfNode shutting down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GladEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
