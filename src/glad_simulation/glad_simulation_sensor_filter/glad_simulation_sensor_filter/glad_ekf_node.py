#!/usr/bin/env python3
"""
G.L.A.D. Simulation 2D Sensor Fusion & Extended Kalman Filter Node.

Fuses:
- Wheel odometry (/motor/odom from Gazebo /odom/wheel)
- IMU (/imu/raw or /imu/data from Gazebo IMU sensor)
- Optional secondary odometry (e.g., /odom_rf2o)

Outputs filtered odometry on /odom and broadcasts odom -> base_link TF.
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
        self.declare_parameter("imu_topic", "/imu/raw")
        self.declare_parameter("output_odom_topic", "odom")

        freq = float(self.get_parameter("frequency").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_link_frame").value
        rf2o_topic = self.get_parameter("odom_rf2o_topic").value
        motor_topic = self.get_parameter("motor_odom_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        out_topic = self.get_parameter("output_odom_topic").value

        # Robot 2D State: [x, y, yaw, vx, vy, vyaw]
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

        self.last_time = self.get_clock().now()
        timer_period = 1.0 / max(1.0, freq)
        self.timer = self.create_timer(timer_period, self.ekf_update)

        self.get_logger().info(
            f"GLAD Simulation EKF initialized at {freq:.1f} Hz. "
            f"Listening to {motor_topic}, {imu_topic}, {rf2o_topic} -> publishing {out_topic}"
        )

    def rf2o_cb(self, msg: Odometry):
        # Update linear velocities if valid
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        if not (math.isnan(vx) or math.isinf(vx)):
            self.vx = 0.7 * self.vx + 0.3 * vx
        if not (math.isnan(vy) or math.isinf(vy)):
            self.vy = 0.7 * self.vy + 0.3 * vy

    def motor_cb(self, msg: Odometry):
        # Wheel odometry provides forward velocity and fallback angular velocity
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        if not (math.isnan(vx) or math.isinf(vx)):
            self.vx = 0.5 * self.vx + 0.5 * vx
        if not (math.isnan(wz) or math.isinf(wz)):
            self.vyaw = 0.3 * self.vyaw + 0.7 * wz

    def imu_cb(self, msg: Imu):
        # Angular velocity z from IMU gyroscope
        gz = msg.angular_velocity.z
        if not (math.isnan(gz) or math.isinf(gz)):
            self.vyaw = 0.8 * gz + 0.2 * self.vyaw

        # Optional yaw extraction from orientation quaternion
        q = msg.orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if norm > 1e-4:
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            imu_yaw = math.atan2(siny_cosp, cosy_cosp)
            # Complementary blend on absolute heading
            yaw_diff = math.atan2(math.sin(imu_yaw - self.yaw), math.cos(imu_yaw - self.yaw))
            self.yaw += 0.05 * yaw_diff

    def ekf_update(self):
        if not rclpy.ok():
            return
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Dead reckoning integration using fused state
        delta_dist = self.vx * dt
        delta_yaw = self.vyaw * dt

        self.yaw = math.atan2(math.sin(self.yaw + delta_yaw), math.cos(self.yaw + delta_yaw))
        self.x += delta_dist * math.cos(self.yaw)
        self.y += delta_dist * math.sin(self.yaw)

        # Build Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        # 6x6 Pose Covariance
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.01   # x
        pose_cov[7] = 0.01   # y
        pose_cov[14] = 999.0 # z
        pose_cov[21] = 999.0 # roll
        pose_cov[28] = 999.0 # pitch
        pose_cov[35] = 0.02  # yaw
        odom_msg.pose.covariance = pose_cov

        # Twist
        odom_msg.twist.twist.linear.x = float(self.vx)
        odom_msg.twist.twist.linear.y = float(self.vy)
        odom_msg.twist.twist.angular.z = float(self.vyaw)

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.02
        twist_cov[7] = 0.02
        twist_cov[14] = 999.0
        twist_cov[21] = 999.0
        twist_cov[28] = 999.0
        twist_cov[35] = 0.05
        odom_msg.twist.covariance = twist_cov

        try:
            self.odom_pub.publish(odom_msg)
        except Exception:
            pass

        # Broadcast odom -> base_link TF
        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
            try:
                self.tf_broadcaster.sendTransform(t)
            except Exception:
                pass


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
