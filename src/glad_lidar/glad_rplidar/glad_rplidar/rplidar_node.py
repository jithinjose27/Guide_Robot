import serial
import struct
import math
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


RPLIDAR_START_FLAG = 0xA5
CMD_STOP = 0x25
CMD_SCAN = 0x20
CMD_FORCE_SCAN = 0x21
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52

RESPONSE_DESCRIPTOR_LEN = 7
SINGLE_RESP_NODE_LEN = 5


class RPLidarNode(Node):
    def __init__(self):
        super().__init__("rplidar_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baudrate", 115200)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("inverted", False)
        self.declare_parameter("angle_compensate", True)
        self.declare_parameter("scan_mode", "Sensitivity")

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("serial_baudrate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.inverted = self.get_parameter("inverted").value

        self.publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.serial = None
        self.running = False
        self.lock = threading.Lock()

        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )
            self.get_logger().info(f"Connected to RPLidar on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            return

        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.scan_thread.start()

    def _send_cmd(self, cmd):
        with self.lock:
            self.serial.write(bytes([RPLIDAR_START_FLAG, cmd]))

    def _read_descriptor(self):
        data = self.serial.read(RESPONSE_DESCRIPTOR_LEN)
        if len(data) < RESPONSE_DESCRIPTOR_LEN:
            return None, None
        resp_type = data[2] if len(data) > 2 else 0
        size_bytes = struct.unpack_from("<I", data, 3)[0] if len(data) >= 7 else 0
        return resp_type, size_bytes

    def _read_response(self, size):
        data = bytearray()
        while len(data) < size:
            chunk = self.serial.read(size - len(data))
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data)

    def _scan_loop(self):
        self._send_cmd(CMD_SCAN)
        time.sleep(0.1)
        self.serial.flushInput()
        self._send_cmd(CMD_SCAN)
        resp_type, size = self._read_descriptor()

        if resp_type is None:
            self.get_logger().error("No response from lidar")
            self.running = False
            return

        self.get_logger().info("RPLidar scan started")

        while self.running and rclpy.ok():
            raw = self._read_response(SINGLE_RESP_NODE_LEN * 100)
            if not raw:
                continue

            n_points = len(raw) // SINGLE_RESP_NODE_LEN
            stamps = []
            angles = []
            ranges = []
            qualities = []

            ts_now = self.get_clock().now()

            for i in range(n_points):
                offset = i * SINGLE_RESP_NODE_LEN
                node = raw[offset : offset + SINGLE_RESP_NODE_LEN]
                if len(node) < SINGLE_RESP_NODE_LEN:
                    break

                q_angle = struct.unpack_from("<H", node, 0)[0]
                quality = (q_angle >> 9) & 0x3F
                angle_raw = (q_angle & 0x1FF) / 64.0
                dist_raw = struct.unpack_from("<H", node, 2)[0] / 4.0 / 1000.0
                flag = struct.unpack_from("<B", node, 4)[0]

                if dist_raw == 0.0 or flag:
                    continue

                angle = math.radians(angle_raw)
                if self.inverted:
                    angle = 2 * math.pi - angle

                stamps.append(ts_now)
                angles.append(angle)
                ranges.append(dist_raw)
                qualities.append(quality)

            if not ranges:
                continue

            angles = np.array(angles, dtype=np.float32)
            ranges = np.array(ranges, dtype=np.float32)
            qualities = np.array(qualities, dtype=np.float32)

            sort_idx = np.argsort(angles)
            angles = angles[sort_idx]
            ranges = ranges[sort_idx]
            qualities = qualities[sort_idx]

            angle_min = float(angles[0])
            angle_max = float(angles[-1])
            n = len(angles)
            angle_inc = (angle_max - angle_min) / max(n - 1, 1)

            if self.inverted:
                angle_min, angle_max = -angle_max, -angle_min

            scan = LaserScan()
            scan.header.stamp = ts_now.to_msg()
            scan.header.frame_id = self.frame_id
            scan.angle_min = angle_min
            scan.angle_max = angle_max
            scan.angle_increment = angle_inc
            scan.time_increment = 0.0
            scan.scan_time = 0.1
            scan.range_min = 0.15
            scan.range_max = 12.0
            scan.ranges = ranges.tolist()
            scan.intensities = qualities.tolist()

            self.publisher.publish(scan)

    def destroy_node(self):
        self.running = False
        thread = getattr(self, "scan_thread", None)
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
        if self.serial and self.serial.is_open:
            try:
                self._send_cmd(CMD_STOP)
                time.sleep(0.1)
                self.serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RPLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
