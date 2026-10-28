#!/usr/bin/env python3
"""
Static Transform Broadcaster for G.L.A.D (Guide Lab Assistance Droid).

Broadcasts fixed coordinate offsets from the robot's reference footprint (base_link)
to attached sensors (laser, imu_link). Parameters can be overridden via ROS parameters
or launch files for custom robot builds.
"""

import math
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll, pitch, yaw in radians to geometry_msgs/Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__("static_tf_broadcaster")

        # Declare configurable parameters with default physical measurements
        self.declare_parameter("base_frame_id", "base_link")

        # Laser transform parameters
        self.declare_parameter("laser_frame_id", "laser")
        self.declare_parameter("laser_x", 0.10)
        self.declare_parameter("laser_y", 0.0)
        self.declare_parameter("laser_z", 0.05)
        self.declare_parameter("laser_roll", 0.0)
        self.declare_parameter("laser_pitch", 0.0)
        self.declare_parameter("laser_yaw", 0.0)

        # IMU transform parameters
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("imu_x", 0.05)
        self.declare_parameter("imu_y", 0.0)
        self.declare_parameter("imu_z", 0.0)
        self.declare_parameter("imu_roll", 0.0)
        self.declare_parameter("imu_pitch", 0.0)
        self.declare_parameter("imu_yaw", 0.0)

        base_frame = self.get_parameter("base_frame_id").value

        laser_frame = self.get_parameter("laser_frame_id").value
        lx = float(self.get_parameter("laser_x").value)
        ly = float(self.get_parameter("laser_y").value)
        lz = float(self.get_parameter("laser_z").value)
        l_roll = float(self.get_parameter("laser_roll").value)
        l_pitch = float(self.get_parameter("laser_pitch").value)
        l_yaw = float(self.get_parameter("laser_yaw").value)

        imu_frame = self.get_parameter("imu_frame_id").value
        ix = float(self.get_parameter("imu_x").value)
        iy = float(self.get_parameter("imu_y").value)
        iz = float(self.get_parameter("imu_z").value)
        i_roll = float(self.get_parameter("imu_roll").value)
        i_pitch = float(self.get_parameter("imu_pitch").value)
        i_yaw = float(self.get_parameter("imu_yaw").value)

        self.broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()

        # 1. Base -> Laser
        laser_tf = TransformStamped()
        laser_tf.header.stamp = now
        laser_tf.header.frame_id = base_frame
        laser_tf.child_frame_id = laser_frame
        laser_tf.transform.translation.x = lx
        laser_tf.transform.translation.y = ly
        laser_tf.transform.translation.z = lz
        laser_tf.transform.rotation = euler_to_quaternion(l_roll, l_pitch, l_yaw)

        # 2. Base -> IMU
        imu_tf = TransformStamped()
        imu_tf.header.stamp = now
        imu_tf.header.frame_id = base_frame
        imu_tf.child_frame_id = imu_frame
        imu_tf.transform.translation.x = ix
        imu_tf.transform.translation.y = iy
        imu_tf.transform.translation.z = iz
        imu_tf.transform.rotation = euler_to_quaternion(i_roll, i_pitch, i_yaw)

        self.broadcaster.sendTransform([laser_tf, imu_tf])
        self.get_logger().info(
            f"Static transforms published: {base_frame} -> {laser_frame} "
            f"({lx:.2f}, {ly:.2f}, {lz:.2f}), {base_frame} -> {imu_frame} "
            f"({ix:.2f}, {iy:.2f}, {iz:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcaster()
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
