#!/usr/bin/env python3
"""
RPLidar Serial Driver for G.L.A.D.

Connects to Slamtec RPLidar (A1/A2 series) over UART serial, decodes
standard 5-byte measurement packets, buffers a complete 360-degree rotation,
and publishes compliant sensor_msgs/LaserScan messages.
"""

import math
import struct
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

RPLIDAR_START_FLAG = 0xA5
CMD_STOP = 0x25
CMD_SCAN = 0x20
CMD_FORCE_SCAN = 0x21
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
CMD_RESET = 0x40

DESCRIPTOR_LEN = 7
NODE_LEN = 5

STATUS_OK = 0
STATUS_WARNING = 1
STATUS_ERROR = 2


class RPLidarNode(Node):
    def __init__(self):
        super().__init__("rplidar_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baudrate", 115200)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("inverted", False)
        self.declare_parameter("angle_compensate", True)
        self.declare_parameter("scan_mode", "Sensitivity")
        self.declare_parameter("scan_topic", "/scan_unfiltered")
        self.declare_parameter("num_samples", 360)
        self.declare_parameter("min_range", 0.15)
        self.declare_parameter("max_range", 12.0)

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("serial_baudrate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.inverted = self.get_parameter("inverted").value
        scan_mode = self.get_parameter("scan_mode").value
        scan_topic = self.get_parameter("scan_topic").value
        self.num_samples = int(self.get_parameter("num_samples").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)

        self.publisher = self.create_publisher(LaserScan, scan_topic, 10)
        self.serial = None
        self.running = False
        self.lock = threading.Lock()

        try:
            # DTR/RTS false prevents pyserial from toggling lines that halt RPLidar motor
            self.serial = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.2,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                dsrdtr=False,
                rtscts=False,
            )
            self.get_logger().info(f"Opened RPLidar serial connection on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(
                f"Failed to open RPLidar port '{port}': {e}. "
                "Check USB connection and dialout permissions (sudo usermod -a -G dialout $USER)."
            )
            return

        # Start motor by de-asserting DTR
        try:
            self.serial.dtr = False
            self.serial.rts = False
        except Exception:
            pass

        time.sleep(0.1)

        if not self._get_device_info():
            self.get_logger().warn("RPLidar GET_INFO unresponsive, proceeding with standard scan mode...")

        self.get_logger().info(f"RPLidar initialized (scan_mode={scan_mode}) -> publishing {scan_topic}")

        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.scan_thread.start()

    def _send_cmd(self, cmd: int):
        with self.lock:
            if self.serial and self.serial.is_open:
                self.serial.reset_input_buffer()
                self.serial.write(bytes([RPLIDAR_START_FLAG, cmd]))

    def _read_exact(self, n: int) -> bytes:
        data = bytearray()
        start = time.time()
        while len(data) < n and self.running:
            if not self.serial or not self.serial.is_open:
                break
            chunk = self.serial.read(n - len(data))
            if chunk:
                data.extend(chunk)
            elif time.time() - start > 1.0:
                break
        return bytes(data)

    def _read_descriptor(self):
        d = self._read_exact(DESCRIPTOR_LEN)
        if len(d) < DESCRIPTOR_LEN or d[0] != RPLIDAR_START_FLAG:
            return None, None
        length = struct.unpack_from("<H", d, 2)[0]
        resp_type = struct.unpack_from("<H", d, 5)[0]
        return resp_type, length

    def _get_device_info(self) -> bool:
        try:
            self._send_cmd(CMD_GET_INFO)
            resp_type, _ = self._read_descriptor()
            if resp_type is None:
                return False
            info = self._read_exact(20)
            if len(info) == 20:
                model = info[0]
                firmware = f"{info[2]}.{info[1]}"
                hardware = info[3]
                serialno = "".join(f"{b:02X}" for b in info[4:20])
                self.get_logger().info(
                    f"RPLidar Info: Model={model}, FW={firmware}, HW={hardware}, S/N={serialno}"
                )
                return True
        except Exception as e:
            self.get_logger().warn(f"GET_INFO failed: {e}")
        return False

    def _scan_loop(self):
        self._send_cmd(CMD_SCAN)
        time.sleep(0.1)
        resp_type, _ = self._read_descriptor()
        if resp_type is None:
            self.get_logger().warn("RPLidar did not acknowledge CMD_SCAN descriptor, retrying...")

        self.get_logger().info("RPLidar scan loop actively capturing measurements")

        # Accumulator for one full 360-degree revolution
        current_angles = []
        current_ranges = []
        current_qualities = []
        scan_start_time = self.get_clock().now()

        while self.running and rclpy.ok():
            try:
                # Read 5-byte measurement node
                node = self._read_exact(NODE_LEN)
                if len(node) < NODE_LEN:
                    continue

                b0, b1, b2, b3, b4 = node[0], node[1], node[2], node[3], node[4]

                # Protocol verification:
                # b0: sync bit (bit 0) and inverse sync bit (bit 1)
                sync = b0 & 0x01
                inv_sync = (b0 >> 1) & 0x01
                quality = b0 >> 2
                checkbit = b1 & 0x01

                # If checkbit != 1, serial stream is misaligned: resynchronize byte by byte
                if checkbit != 1 or (sync == inv_sync):
                    single_byte = self.serial.read(1)
                    continue

                # Valid node: decode angle and distance
                angle_q6 = (b1 >> 1) | (b2 << 7)
                angle_deg = angle_q6 / 64.0
                dist_q2 = b3 | (b4 << 8)
                dist_m = (dist_q2 / 4.0) / 1000.0

                # Check if this node marks the start of a new 360-degree revolution
                if sync == 1 and len(current_ranges) > 50:
                    # Publish accumulated scan
                    self._publish_scan(
                        scan_start_time,
                        current_angles,
                        current_ranges,
                        current_qualities,
                    )
                    # Reset buffers for new revolution
                    current_angles = []
                    current_ranges = []
                    current_qualities = []
                    scan_start_time = self.get_clock().now()

                if dist_m > 0.0:
                    angle_rad = math.radians(angle_deg)
                    # Normalize angle to [-pi, pi]
                    angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
                    if self.inverted:
                        angle_rad = -angle_rad

                    current_angles.append(angle_rad)
                    current_ranges.append(dist_m)
                    current_qualities.append(float(quality))

            except Exception as e:
                if self.running:
                    self.get_logger().warn(f"RPLidar read error: {e}")
                time.sleep(0.01)

    def _publish_scan(self, start_time, angles, ranges, qualities):
        if not ranges:
            return

        # Uniform angular grid covering [-pi, pi]
        angle_min = -math.pi
        angle_max = math.pi
        num_bins = self.num_samples
        angle_inc = (angle_max - angle_min) / float(num_bins)

        grid_ranges = [float("inf")] * num_bins
        grid_intensities = [0.0] * num_bins

        # Populate nearest bin
        for ang, dist, qual in zip(angles, ranges, qualities):
            if self.min_range <= dist <= self.max_range:
                idx = int(round((ang - angle_min) / angle_inc))
                if 0 <= idx < num_bins:
                    # Keep closest obstacle in bin
                    if dist < grid_ranges[idx]:
                        grid_ranges[idx] = float(dist)
                        grid_intensities[idx] = float(qual)

        scan = LaserScan()
        scan.header.stamp = start_time.to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1  # ~10 Hz rotation rate
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = grid_ranges
        scan.intensities = grid_intensities

        self.publisher.publish(scan)

    def destroy_node(self):
        self.running = False
        thread = getattr(self, "scan_thread", None)
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
        if self.serial and self.serial.is_open:
            try:
                self._send_cmd(CMD_STOP)
                time.sleep(0.05)
                self.serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RPLidarNode()
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
