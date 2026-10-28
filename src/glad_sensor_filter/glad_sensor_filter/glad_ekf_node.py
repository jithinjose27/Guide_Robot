#!/usr/bin/env python3
"""
G.L.A.D. 2D Sensor Fusion & Extended Kalman Filter Node.

Combines:
- Laser odometry (/odom_rf2o) for translational velocity (vx, vy)
- Wheel odometry (/motor/odom) for drive baseline velocity
- IMU (/imu/data) for angular velocity (gz) and heading yaw

Produces filtered odometry on /odom (or odometry/filtered) and broadcasts
the continuous odom -> base_link TF coordinate frame.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster


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
        self.declare_parameter("odom_rf2o_topic", "/odom_rf2o")
        self.declare_parameter("motor_odom_topic", "/motor/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("output_odom_topic", "odom")

        freq = float(self.get_parameter("frequency").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_link_frame").value
        rf2o_topic = self.get_parameter("odom_rf2o_topic").value
        motor_topic = self.get_parameter("motor_odom_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        out_topic = self.get_parameter("output_odom_topic").value

        # Robot 2D State: [x, y, yaw, vx, vyaw]
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        # Subscriptions
        self.sub_rf2o = self.create_subscription(
            Odometry, rf2o_topic, self.rf2o_cb, 10
        )
        self.sub_motor = self.create_subscription(
            Odometry, motor_topic, self.motor_cb, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, imu_topic, self.imu_cb, 10
        )

        # Publisher & TF
        self.odom_pub = self.create_publisher(Odometry, out_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.last_update_time = self.get_clock().now()
        timer_period = 1.0 / max(1.0, freq)
        self.timer = self.create_timer(timer_period, self.update_filter)

        self.get_logger().info(
            f"GladEkfNode active @ {freq}Hz: fusing {rf2o_topic}, {motor_topic}, {imu_topic} -> {out_topic}"
        )

    def rf2o_cb(self, msg: Odometry):
        # Update forward velocity from laser odometry
        lx = msg.twist.twist.linear.x
        ly = msg.twist.twist.linear.y
        if math.isfinite(lx):
            self.vx = float(lx)
        if math.isfinite(ly):
            self.vy = float(ly)

    def motor_cb(self, msg: Odometry):
        # If laser odometry is absent or zero, fall back to wheel odometry
        if abs(self.vx) < 1e-4:
            mx = msg.twist.twist.linear.x
            if math.isfinite(mx):
                self.vx = float(mx)

    def imu_cb(self, msg: Imu):
        # Angular rate z
        gz = msg.angular_velocity.z
        if math.isfinite(gz):
            self.vyaw = float(gz)

        # Orientation yaw extraction from quaternion
        q = msg.orientation
        if abs(q.w) > 1e-4 or abs(q.z) > 1e-4:
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            imu_yaw = math.atan2(siny_cosp, cosy_cosp)
            # Gentle complementary filter blend towards IMU heading
            yaw_diff = math.atan2(math.sin(imu_yaw - self.yaw), math.cos(imu_yaw - self.yaw))
            self.yaw += 0.05 * yaw_diff

    def update_filter(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Dead reckoning kinematic state propagation
        self.yaw += self.vyaw * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        dx = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        dy = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt

        self.x += dx
        self.y += dy

        q = self._yaw_to_quaternion(self.yaw)

        # Publish filtered Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose = Pose(
            position=Point(x=float(self.x), y=float(self.y), z=0.0),
            orientation=q,
        )
        odom.twist.twist = Twist(
            linear=Vector3(x=float(self.vx), y=float(self.vy), z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(self.vyaw)),
        )

        pose_cov = [0.0] * 36
        pose_cov[0] = 0.01   # x
        pose_cov[7] = 0.01   # y
        pose_cov[14] = 1e6
        pose_cov[21] = 1e6
        pose_cov[28] = 1e6
        pose_cov[35] = 0.02  # yaw
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.01  # vx
        twist_cov[7] = 0.01  # vy
        twist_cov[14] = 1e6
        twist_cov[21] = 1e6
        twist_cov[28] = 1e6
        twist_cov[35] = 0.02 # vyaw
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        # Broadcast odom -> base_link
        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        half = yaw * 0.5
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def main(args=None):
    rclpy.init(args=args)
    node = GladEkfNode()
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
