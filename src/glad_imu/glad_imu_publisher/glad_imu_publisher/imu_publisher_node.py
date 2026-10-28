#!/usr/bin/env python3
"""
BNO055 9-DOF IMU Hardware Driver for G.L.A.D.

Reads gyroscope, accelerometer, and magnetometer vectors over I2C,
applies calibration offsets and noise deadzones, and publishes raw ROS 2 topics:
- /imu/data_raw (sensor_msgs/Imu)
- /imu/mag (sensor_msgs/MagneticField)
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from ament_index_python.packages import get_package_share_directory


class BNO055Driver(Node):
    def __init__(self):
        super().__init__("bno055_driver")

        pkg_share = get_package_share_directory("glad_imu_publisher")
        default_config = os.path.join(pkg_share, "config", "bno055_config.yaml")
        self.declare_parameter("config_file", default_config)
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("frame_id", "imu_link")

        config_path = self.get_parameter("config_file").value
        rate = float(self.get_parameter("rate").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.offsets = {
            "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
            "mag": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
        self.deadzones = {"gyro": 0.0, "accel": 0.0}

        try:
            if os.path.exists(config_path):
                with open(config_path, "r") as f:
                    cfg = yaml.safe_load(f)
                    if "gyro_offsets" in cfg:
                        self.offsets["gyro"] = cfg["gyro_offsets"]
                    if "mag_offsets" in cfg:
                        self.offsets["mag"] = cfg["mag_offsets"]
                    if "deadzones" in cfg:
                        self.deadzones = cfg["deadzones"]
                    self.get_logger().info(f"Loaded IMU calibration parameters from {config_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not load IMU config from {config_path}: {e}")

        self.sensor = None
        try:
            import board
            import busio
            import adafruit_bno055

            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            self.get_logger().info("Connected to BNO055 IMU sensor over I2C bus")
        except Exception as e:
            self.get_logger().warn(
                f"BNO055 IMU hardware not detected over I2C ({e}). "
                "Ensure I2C is enabled (sudo raspi-config -> Interfaces -> I2C) "
                "and sensor SCL/SDA are connected. Node will remain active for pipeline compatibility."
            )

        self.imu_raw_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 10)

        timer_period = 1.0 / max(1.0, rate)
        self.timer = self.create_timer(timer_period, self.update_sensor)

    @staticmethod
    def apply_deadzone(val: float, threshold: float) -> float:
        if abs(val) < threshold:
            return 0.0
        return val

    def update_sensor(self):
        if self.sensor is None:
            return

        try:
            gyro = self.sensor.gyro
            accel = self.sensor.acceleration
            mag = self.sensor.magnetic

            if None in gyro or None in accel or None in mag:
                return

            now = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.frame_id

            gx = self.apply_deadzone(
                gyro[0] - self.offsets["gyro"].get("x", 0.0), self.deadzones.get("gyro", 0.0)
            )
            gy = -self.apply_deadzone(
                gyro[1] - self.offsets["gyro"].get("y", 0.0), self.deadzones.get("gyro", 0.0)
            )
            gz = -self.apply_deadzone(
                gyro[2] - self.offsets["gyro"].get("z", 0.0), self.deadzones.get("gyro", 0.0)
            )

            ax = -self.apply_deadzone(accel[0], self.deadzones.get("accel", 0.0))
            ay = self.apply_deadzone(accel[1], self.deadzones.get("accel", 0.0))
            az = self.apply_deadzone(accel[2], self.deadzones.get("accel", 0.0))

            imu_msg.angular_velocity.x = float(gx)
            imu_msg.angular_velocity.y = float(gy)
            imu_msg.angular_velocity.z = float(gz)
            imu_msg.linear_acceleration.x = float(ax)
            imu_msg.linear_acceleration.y = float(ay)
            imu_msg.linear_acceleration.z = float(az)

            # Orientation not provided by raw driver
            imu_msg.orientation_covariance[0] = -1.0
            imu_msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ]
            imu_msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1,
            ]

            self.imu_raw_pub.publish(imu_msg)

            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self.frame_id

            raw_mag_x = (mag[0] - self.offsets["mag"].get("x", 0.0)) * 1e-6
            raw_mag_y = (mag[1] - self.offsets["mag"].get("y", 0.0)) * 1e-6
            raw_mag_z = (mag[2] - self.offsets["mag"].get("z", 0.0)) * 1e-6

            mag_msg.magnetic_field.x = float(raw_mag_x)
            mag_msg.magnetic_field.y = float(-raw_mag_y)
            mag_msg.magnetic_field.z = float(-raw_mag_z)

            self.mag_pub.publish(mag_msg)
        except Exception as e:
            self.get_logger().warn(f"IMU sample read failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Driver()
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
