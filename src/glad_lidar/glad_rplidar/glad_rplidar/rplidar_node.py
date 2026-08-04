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

CMD_STOP = 0x25  # stop
CMD_SCAN = 0x20  # request continuous scan
CMD_FORCE_SCAN = 0x21  # force scan (skip motor/check)
CMD_GET_INFO = 0x50  # device info
CMD_GET_HEALTH = 0x52  # device health
CMD_RESET = 0x40  # reset

DESCRIPTOR_LEN = 7
NODE_LEN = 5  # bytes per measurement node

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

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("serial_baudrate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.inverted = self.get_parameter("inverted").value
        scan_mode = self.get_parameter("scan_mode").value

        self.publisher = self.create_publisher(
            LaserScan, self.get_parameter("scan_topic").value, 10
        )
        self.serial = None
        self.running = False
        self.lock = threading.Lock()

        try:
            # dsrdtr=False / rtscts=False prevent pyserial from auto-toggling
            # control lines during open, which stops the RPLidar motor.
            self.serial = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                dsrdtr=False,
                rtscts=False,
            )
            self.get_logger().info(f"Opened {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            return

        # Enable the motor. On the RPLidar A1 (and most SLAMTEC USB adapters)
        # the motor-enable line is wired to the serial DTR/RTS lines.
        # Deasserting DTR/RTS turns the motor on; asserting resets/stops it.
        self.serial.dtr = False
        self.serial.rts = False

        if not self._get_device_info():
            self.get_logger().error("Lidar not responding to GET_INFO")
            self.destroy_node()
            return

        if not self._get_device_health():
            self.get_logger().error("Lidar not responding to GET_HEALTH")
            self.destroy_node()
            return

        # scan_mode switching requires model-specific extended config commands
        # (A2M8/A3 only). The A1 ignores this; send nothing to avoid a hang.
        self.get_logger().info(f"scan_mode={scan_mode} (A1 uses standard scan)")

        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.scan_thread.start()

    # --- low-level serial helpers -------------------------------------------

    def _send_cmd(self, cmd):
        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write(bytes([RPLIDAR_START_FLAG, cmd]))

    def _read_exact(self, n):
        data = bytearray()
        while len(data) < n:
            chunk = self.serial.read(n - len(data))
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data)

    def _read_descriptor(self):
        # 7-byte descriptor: [0]=0xA5 [1]=mode [2:4]=len(LE)
        # [4]=send_mode [5:7]=type(LE)
        d = self._read_exact(DESCRIPTOR_LEN)
        if len(d) < DESCRIPTOR_LEN or d[0] != RPLIDAR_START_FLAG:
            return None, None
        length = struct.unpack_from("<H", d, 2)[0]
        resp_type = struct.unpack_from("<H", d, 5)[0]
        return resp_type, length

    def _get_device_info(self):
        self._send_cmd(CMD_GET_INFO)
        resp_type, length = self._read_descriptor()
        if resp_type is None:
            return False
        info = self._read_exact(20)
        if len(info) == 20:
            model = info[0]
            firmware = info[2]
            hardware = info[3]
            serialno = "".join(f"{b:02X}" for b in info[4:20])
            self.get_logger().info(
                f"Lidar info: model={model} fw={firmware} hw={hardware} "
                f"serial={serialno}"
            )
            return True
        return False

    def _get_device_health(self):
        self._send_cmd(CMD_GET_HEALTH)
        resp_type, length = self._read_descriptor()
        if resp_type is None:
            return False
        health = self._read_exact(3)
        if len(health) == 3:
            status = health[0]
            err = (health[2] << 8) | health[1]
            names = {STATUS_OK: "OK", STATUS_WARNING: "WARNING", STATUS_ERROR: "ERROR"}
            self.get_logger().info(
                f"Lidar health: {names.get(status, 'UNKNOWN')} (err={err})"
            )
            return True
        return False

    # --- scanning -----------------------------------------------------------

    def _parse_nodes(self, raw):
        stamps, angles, ranges, qualities = [], [], [], []
        ts_now = self.get_clock().now()

        n = len(raw) // NODE_LEN
        for i in range(n):
            node = raw[i * NODE_LEN : (i + 1) * NODE_LEN]
            if len(node) < NODE_LEN:
                break
            q_angle = struct.unpack_from("<H", node, 0)[0]
            quality = (q_angle >> 9) & 0x3F
            angle_raw = (q_angle & 0x1FF) / 64.0
            dist_raw = struct.unpack_from("<H", node, 2)[0] / 4.0 / 1000.0
            flag = node[4]

            if dist_raw == 0.0 or flag & 0x01:  # skip dist=0, new scan flag
                continue

            angle = math.radians(angle_raw)
            if self.inverted:
                angle = 2 * math.pi - angle

            stamps.append(ts_now)
            angles.append(angle)
            ranges.append(dist_raw)
            qualities.append(quality)
        return stamps, angles, ranges, qualities

    def _scan_loop(self):
        self._send_cmd(CMD_SCAN)
        time.sleep(0.1)
        resp_type, length = self._read_descriptor()
        if resp_type is None:
            self.get_logger().error("No scan response from lidar")
            self.running = False
            return
        self.get_logger().info("RPLidar scan started")

        while self.running and rclpy.ok():
            raw = self._read_exact(NODE_LEN * 200)
            if not raw:
                continue

            stamps, angles, ranges, qualities = self._parse_nodes(raw)
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
            scan.header.stamp = stamps[0].to_msg()
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
