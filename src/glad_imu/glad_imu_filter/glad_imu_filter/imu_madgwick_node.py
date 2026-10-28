#!/usr/bin/env python3
"""
Madgwick Orientation Filter for G.L.A.D IMU Pipeline.

Fuses angular rates from gyroscope, gravity vector from accelerometer,
and optional magnetic field from magnetometer using gradient descent on
quaternions. Produces drift-compensated, noise-robust 3D orientation.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion


class MadgwickFilter:
    def __init__(self, beta=0.1):
        self.beta = beta
        # Quaternion: [w, x, y, z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz, dt):
        q = self.q
        beta = self.beta

        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if a_norm < 0.001:
            return
        ax /= a_norm
        ay /= a_norm
        az /= a_norm

        q1, q2, q3, q4 = q[0], q[1], q[2], q[3]

        # --- Accelerometer objective function f_g ---
        f_g1 = 2.0 * (q2 * q4 - q1 * q3) - ax
        f_g2 = 2.0 * (q1 * q2 + q3 * q4) - ay
        f_g3 = 2.0 * (0.5 - q2 * q2 - q3 * q3) - az

        # --- Accelerometer Jacobian J_g ---
        J_g11 = -2.0 * q3
        J_g12 = 2.0 * q4
        J_g13 = -2.0 * q1
        J_g14 = 2.0 * q2
        J_g21 = 2.0 * q2
        J_g22 = 2.0 * q1
        J_g23 = 2.0 * q4
        J_g24 = 2.0 * q3
        J_g31 = 0.0
        J_g32 = -4.0 * q2
        J_g33 = -4.0 * q3
        J_g34 = 0.0

        step = np.array(
            [
                J_g11 * f_g1 + J_g21 * f_g2 + J_g31 * f_g3,
                J_g12 * f_g1 + J_g22 * f_g2 + J_g32 * f_g3,
                J_g13 * f_g1 + J_g23 * f_g2 + J_g33 * f_g3,
                J_g14 * f_g1 + J_g24 * f_g2 + J_g34 * f_g3,
            ],
            dtype=np.float64,
        )

        # --- Magnetometer incorporation ---
        m_norm = math.sqrt(mx * mx + my * my + mz * mz)
        if m_norm > 0.001:
            mx /= m_norm
            my /= m_norm
            mz /= m_norm

            hx = (
                2.0 * mx * (0.5 - q3 * q3 - q4 * q4)
                + 2.0 * my * (q1 * q4 + q2 * q3)
                + 2.0 * mz * (q2 * q4 - q1 * q3)
            )
            hy = (
                2.0 * mx * (q2 * q3 - q1 * q4)
                + 2.0 * my * (0.5 - q2 * q2 - q4 * q4)
                + 2.0 * mz * (q1 * q2 + q3 * q4)
            )
            bx = math.sqrt(hx * hx + hy * hy)
            bz = (
                2.0 * mx * (q1 * q3 + q2 * q4)
                + 2.0 * my * (q3 * q4 - q1 * q2)
                + 2.0 * mz * (0.5 - q2 * q2 - q3 * q3)
            )

            f_b1 = (
                2.0 * bx * (0.5 - q3 * q3 - q4 * q4)
                + 2.0 * bz * (q2 * q4 - q1 * q3)
                - mx
            )
            f_b2 = 2.0 * bx * (q2 * q3 - q1 * q4) + 2.0 * bz * (q1 * q2 + q3 * q4) - my
            f_b3 = (
                2.0 * bx * (q1 * q3 + q2 * q4)
                + 2.0 * bz * (0.5 - q2 * q2 - q3 * q3)
                - mz
            )

            J_b11 = -2.0 * bz * q3
            J_b12 = 2.0 * bz * q4
            J_b13 = -4.0 * bx * q3 - 2.0 * bz * q1
            J_b14 = -4.0 * bx * q4 + 2.0 * bz * q2
            J_b21 = -2.0 * bx * q4 + 2.0 * bz * q2
            J_b22 = 2.0 * bx * q3 + 2.0 * bz * q1
            J_b23 = 2.0 * bx * q2 + 2.0 * bz * q4
            J_b24 = -2.0 * bx * q1 + 2.0 * bz * q3
            J_b31 = 2.0 * bx * q3
            J_b32 = 2.0 * bx * q4 - 4.0 * bz * q2
            J_b33 = 2.0 * bx * q1 - 4.0 * bz * q3
            J_b34 = 2.0 * bx * q2

            step[0] += J_b11 * f_b1 + J_b21 * f_b2 + J_b31 * f_b3
            step[1] += J_b12 * f_b1 + J_b22 * f_b2 + J_b32 * f_b3
            step[2] += J_b13 * f_b1 + J_b23 * f_b2 + J_b33 * f_b3
            step[3] += J_b14 * f_b1 + J_b24 * f_b2 + J_b34 * f_b3

        step_norm = np.linalg.norm(step)
        if step_norm > 0:
            step /= step_norm

        # Gyroscope quaternion derivative: 0.5 * q ⊗ ω
        q_dot = 0.5 * np.array(
            [
                -q2 * gx - q3 * gy - q4 * gz,
                q1 * gx + q3 * gz - q4 * gy,
                q1 * gy - q2 * gz + q4 * gx,
                q1 * gz + q2 * gy - q3 * gx,
            ],
            dtype=np.float64,
        )

        q_dot -= beta * step

        q += q_dot * dt
        q_norm = np.linalg.norm(q)
        if q_norm > 0:
            q /= q_norm

        self.q = q

    def get_quaternion(self):
        return self.q


class ImuMadgwickNode(Node):
    def __init__(self):
        super().__init__("imu_madgwick_node")

        self.declare_parameter("beta", 0.1)
        self.declare_parameter("use_mag", True)
        self.declare_parameter("world_frame", "enu")
        self.declare_parameter("fixed_frame", "odom")

        beta = float(self.get_parameter("beta").value)
        self.use_mag = bool(self.get_parameter("use_mag").value)
        self.world_frame = self.get_parameter("world_frame").value
        self.fixed_frame = self.get_parameter("fixed_frame").value

        self.filter = MadgwickFilter(beta=beta)

        self.imu_sub = self.create_subscription(
            Imu, "/imu/data_raw", self.imu_callback, 10
        )
        self.mag_sub = self.create_subscription(
            MagneticField, "/imu/mag", self.mag_callback, 10
        )

        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)

        self.mag_data = None
        self.last_time = None

        self.get_logger().info(
            f"Madgwick orientation filter started (beta={beta}, use_mag={self.use_mag})"
        )

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Clamping dt prevents numeric explosion during system hiccups
        if dt <= 0:
            return
        dt = min(0.1, dt)

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        mx = my = mz = 0.0
        if self.use_mag and self.mag_data is not None:
            mx = self.mag_data.magnetic_field.x
            my = self.mag_data.magnetic_field.y
            mz = self.mag_data.magnetic_field.z

        self.filter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt)
        q = self.filter.get_quaternion()

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id or "imu_link"

        # q is [w, x, y, z] -> map to geometry_msgs Quaternion
        out.orientation = Quaternion(x=float(q[1]), y=float(q[2]), z=float(q[3]), w=float(q[0]))
        out.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]

        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.imu_pub.publish(out)

    def mag_callback(self, msg: MagneticField):
        self.mag_data = msg


def main(args=None):
    rclpy.init(args=args)
    node = ImuMadgwickNode()
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
