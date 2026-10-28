#!/usr/bin/env python3
"""
ESP32 Differential Drive Motor Bridge for G.L.A.D.

Connects to ESP32 motor driver board via UART serial:
- Converts /cmd_vel (Twist) -> differential wheel velocities -> PWM commands sent over serial.
- Reads encoder tick counts over serial -> calculates wheel odometry -> publishes /motor/odom.
"""

import math
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3
from nav_msgs.msg import Odometry
import serial


class ESP32MotorBridge(Node):
    def __init__(self):
        super().__init__("esp32_motor_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("wheel_radius", 0.04)
        self.declare_parameter("wheel_separation", 0.325)
        self.declare_parameter("ticks_per_rev", 996.8)
        self.declare_parameter("max_pwm", 255.0)
        self.declare_parameter("max_rpm", 100.0)
        self.declare_parameter("odom_topic", "/motor/odom")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        self.max_pwm = float(self.get_parameter("max_pwm").value)
        self.max_rpm = float(self.get_parameter("max_rpm").value)
        odom_topic = self.get_parameter("odom_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("baud_rate").value)

        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 motor driver on {port} @ {baud}")
        except Exception as e:
            self.get_logger().warn(
                f"Could not open ESP32 serial port '{port}': {e}. "
                "Ensure ESP32 is plugged in and user is in dialout group (sudo usermod -a -G dialout $USER). "
                "Node will remain online in simulation/dry-run mode."
            )

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0
        self.last_odom_time = self.get_clock().now()

        # Publishers & Subscribers
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        # Background thread for continuous serial reads
        self.lock = threading.Lock()
        self.latest_ticks = [0, 0]
        self.stop_thread = False

        if self.ser and self.ser.is_open:
            self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.read_thread.start()

        # 20 Hz odometry loop
        self.create_timer(0.05, self.update_odometry)
        self.get_logger().info(f"ESP32 motor bridge active -> publishing {odom_topic}")

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist velocity into individual motor PWM and send over serial."""
        if not self.ser or not self.ser.is_open:
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive inverse kinematics
        vel_right = linear_x + (angular_z * self.wheel_separation / 2.0)
        vel_left = linear_x - (angular_z * self.wheel_separation / 2.0)

        # Velocity (m/s) -> RPM
        circumference = 2.0 * math.pi * self.wheel_radius
        rpm_right = (vel_right / circumference) * 60.0
        rpm_left = (vel_left / circumference) * 60.0

        # RPM -> PWM mapping
        pwm_right = int((rpm_right / self.max_rpm) * self.max_pwm)
        pwm_left = int((rpm_left / self.max_rpm) * self.max_pwm)

        # Clamp to [-255, 255]
        pwm_right = max(-int(self.max_pwm), min(int(self.max_pwm), pwm_right))
        pwm_left = max(-int(self.max_pwm), min(int(self.max_pwm), pwm_left))

        command = f"{pwm_right},{pwm_left}\n"
        try:
            self.ser.write(command.encode("utf-8"))
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def read_serial_loop(self):
        """Continuously parse incoming tick counts from ESP32."""
        while not self.stop_thread and rclpy.ok():
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if "," in line:
                        parts = line.split(",")
                        if len(parts) == 2:
                            with self.lock:
                                try:
                                    self.latest_ticks[0] = int(parts[1])
                                    self.latest_ticks[1] = int(parts[0])
                                except ValueError:
                                    pass
            except Exception:
                pass
            time.sleep(0.005)

    def update_odometry(self):
        """Calculate forward kinematics from encoder ticks and publish Odometry."""
        current_time = self.get_clock().now()

        with self.lock:
            curr_ticks_left = self.latest_ticks[0]
            curr_ticks_right = self.latest_ticks[1]

        d_ticks_left = curr_ticks_left - self.prev_ticks_left
        d_ticks_right = curr_ticks_right - self.prev_ticks_right

        self.prev_ticks_left = curr_ticks_left
        self.prev_ticks_right = curr_ticks_right

        circumference = 2.0 * math.pi * self.wheel_radius
        d_left = (d_ticks_left / self.ticks_per_rev) * circumference
        d_right = (d_ticks_right / self.ticks_per_rev) * circumference

        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)
        self.th += d_theta
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time

        vx = d_center / dt if dt > 0.0 else 0.0
        vth = d_theta / dt if dt > 0.0 else 0.0

        q = self.euler_to_quaternion(0.0, 0.0, self.th)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose = Pose(
            position=Point(x=float(self.x), y=float(self.y), z=0.0),
            orientation=q,
        )
        odom.twist.twist = Twist(
            linear=Vector3(x=float(vx), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(vth)),
        )

        cov = [0.0] * 36
        cov[0] = 0.05
        cov[7] = 0.05
        cov[14] = 1e6
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = 0.1
        odom.pose.covariance = cov
        odom.twist.covariance = cov

        self.odom_pub.publish(odom)

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        half_yaw = yaw * 0.5
        return Quaternion(x=0.0, y=0.0, z=math.sin(half_yaw), w=math.cos(half_yaw))

    def on_shutdown(self):
        self.stop_thread = True
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"0,0\n")
                time.sleep(0.05)
                self.ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ESP32MotorBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
