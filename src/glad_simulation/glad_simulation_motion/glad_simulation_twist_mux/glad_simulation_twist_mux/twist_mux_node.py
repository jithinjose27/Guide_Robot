#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class TwistMux(Node):
    def __init__(self):
        super().__init__("twist_mux")

        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("priority_stop", 100)
        self.declare_parameter("priority_teleop", 70)
        self.declare_parameter("priority_nav", 40)

        self.timeout = self.get_parameter("timeout").value
        self.priorities = {
            "stop": self.get_parameter("priority_stop").value,
            "teleop": self.get_parameter("priority_teleop").value,
            "nav": self.get_parameter("priority_nav").value,
        }

        self.stop_active = False
        self.teleop_twist = Twist()
        self.nav_twist = Twist()

        self.stop_stamp = self.get_clock().now()
        self.teleop_stamp = self.get_clock().now()
        self.nav_stamp = self.get_clock().now()

        self.pub = self.create_publisher(Twist, "/cmd_vel/raw", 10)

        self.create_subscription(Bool, "/cmd_vel/stop", self.stop_cb, 10)
        self.create_subscription(Twist, "/cmd_vel/teleop", self.teleop_cb, 10)
        self.create_subscription(Twist, "/cmd_vel/nav", self.nav_cb, 10)

        self.create_timer(0.1, self.publish_cmd)

        sorted_priorities = sorted(
            self.priorities.items(), key=lambda x: x[1], reverse=True
        )
        priority_str = ", ".join(f"{name}={prio}" for name, prio in sorted_priorities)
        self.get_logger().info(f"Twist mux started — priorities: {priority_str}")

    def stop_cb(self, msg):
        self.stop_active = msg.data
        self.stop_stamp = self.get_clock().now()

    def teleop_cb(self, msg):
        self.teleop_twist = msg
        self.teleop_stamp = self.get_clock().now()

    def nav_cb(self, msg):
        self.nav_twist = msg
        self.nav_stamp = self.get_clock().now()

    def publish_cmd(self):
        now = self.get_clock().now()
        stop_age = (now - self.stop_stamp).nanoseconds / 1e9
        teleop_age = (now - self.teleop_stamp).nanoseconds / 1e9
        nav_age = (now - self.nav_stamp).nanoseconds / 1e9

        ages = {
            "stop": stop_age,
            "teleop": teleop_age,
            "nav": nav_age,
        }

        sorted_sources = sorted(
            self.priorities.items(), key=lambda x: x[1], reverse=True
        )

        for source, _ in sorted_sources:
            if ages[source] < self.timeout:
                if source == "stop":
                    if self.stop_active:
                        self.pub.publish(Twist())
                        return
                elif source == "teleop":
                    self.pub.publish(self.teleop_twist)
                    return
                elif source == "nav":
                    self.pub.publish(self.nav_twist)
                    return

        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TwistMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
