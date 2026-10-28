#!/usr/bin/env python3
"""
Laser Filter & Resampler Node for G.L.A.D.

Applies:
1. Angular field-of-view filtering (e.g. keeping forward 180 degrees)
2. Range bounds clipping (min_range to max_range)
3. Uniform beam resampling with exact angular fidelity
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserFilterNode(Node):
    def __init__(self):
        super().__init__("laser_filter_node")

        self.declare_parameter("input_scan_topic", "/scan_unfiltered")
        self.declare_parameter("output_scan_topic", "/scan")
        self.declare_parameter("target_beams", 360)

        self.declare_parameter("filter1.name", "angle")
        self.declare_parameter("filter1.type", "angular_bounds")
        self.declare_parameter("filter1.lower_angle", -1.5708)
        self.declare_parameter("filter1.upper_angle", 1.5708)

        self.declare_parameter("filter2.name", "range")
        self.declare_parameter("filter2.type", "range_filter")
        self.declare_parameter("filter2.min_range", 0.20)
        self.declare_parameter("filter2.max_range", 12.0)

        input_topic = self.get_parameter("input_scan_topic").value
        output_topic = self.get_parameter("output_scan_topic").value
        self.target_beams = int(self.get_parameter("target_beams").value)

        self.lower_angle = float(self.get_parameter("filter1.lower_angle").value)
        self.upper_angle = float(self.get_parameter("filter1.upper_angle").value)

        self.min_range = float(self.get_parameter("filter2.min_range").value)
        self.max_range = float(self.get_parameter("filter2.max_range").value)

        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, 10
        )
        self.publisher = self.create_publisher(LaserScan, output_topic, 10)

        self.get_logger().info(
            f"LaserFilterNode active: {input_topic} -> {output_topic} "
            f"([ {self.lower_angle:.2f}, {self.upper_angle:.2f} ] rad, "
            f"[ {self.min_range:.2f}, {self.max_range:.2f} ] m)"
        )

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        n_input = len(ranges)
        if n_input == 0:
            return

        intensities = (
            np.array(msg.intensities, dtype=np.float32)
            if len(msg.intensities) == n_input
            else None
        )

        # 1. Replace NaN and negative values with +inf
        invalid_mask = np.isnan(ranges) | np.isneginf(ranges)
        ranges[invalid_mask] = float("inf")
        if intensities is not None:
            intensities[invalid_mask] = 0.0

        # 2. Angle Bounds Filter
        angles = msg.angle_min + np.arange(n_input) * msg.angle_increment
        angles_norm = np.arctan2(np.sin(angles), np.cos(angles))
        lower = min(self.lower_angle, self.upper_angle)
        upper = max(self.lower_angle, self.upper_angle)

        outside_fov = (angles_norm < lower) | (angles_norm > upper)
        ranges[outside_fov] = float("inf")
        if intensities is not None:
            intensities[outside_fov] = 0.0

        # 3. Range Filter (Too close or too far -> +inf)
        out_of_range = (ranges < self.min_range) | (ranges > self.max_range)
        ranges[out_of_range] = float("inf")
        if intensities is not None:
            intensities[out_of_range] = 0.0

        # 4. Resample if target_beams is specified and different from n_input
        target = self.target_beams if self.target_beams > 0 else n_input
        if n_input != target:
            target_angles = np.linspace(msg.angle_min, msg.angle_max, target)
            # Find nearest corresponding beam to preserve distance accuracy
            indices = np.clip(
                np.round((target_angles - msg.angle_min) / msg.angle_increment).astype(int),
                0,
                n_input - 1,
            )
            final_ranges = ranges[indices]
            final_intensities = intensities[indices] if intensities is not None else None
            angle_inc = (msg.angle_max - msg.angle_min) / float(max(1, target - 1))
        else:
            final_ranges = ranges
            final_intensities = intensities
            angle_inc = msg.angle_increment

        filtered = LaserScan()
        filtered.header = msg.header
        filtered.header.stamp = self.get_clock().now().to_msg()
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = angle_inc
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = self.min_range
        filtered.range_max = self.max_range
        filtered.ranges = final_ranges.tolist()
        filtered.intensities = (
            final_intensities.tolist() if final_intensities is not None else []
        )

        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
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
