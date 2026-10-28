#!/usr/bin/env python3
"""
Range-Flow / Scan-Matching Laser Odometry for G.L.A.D.

Estimates 2D planar velocity and accumulated pose by matching consecutive
laser scan returns via SVD-based rigid transform alignment (Kabsch algorithm).
Publishes nav_msgs/Odometry for sensor fusion in robot_localization / EKF.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster


class RF2oOdometryNode(Node):
    def __init__(self):
        super().__init__("rf2o_odometry_node")

        self.declare_parameter("laser_scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom_rf2o")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("freq", 20.0)

        scan_topic = self.get_parameter("laser_scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value

        self.subscription = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.prev_ranges = None
        self.prev_angle_min = None
        self.prev_angle_increment = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None

        self.get_logger().info(
            f"RF2oOdometryNode active: {scan_topic} -> {odom_topic} (publish_tf={self.publish_tf})"
        )

    def scan_callback(self, msg: LaserScan):
        current_time = self.get_clock().now()
        ranges = np.array(msg.ranges, dtype=np.float32)
        n_pts = len(ranges)
        if n_pts < 20:
            return

        angles = msg.angle_min + np.arange(n_pts) * msg.angle_increment

        # Valid mask
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

        if self.prev_ranges is not None and self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds * 1e-9
            if 0.001 < dt < 1.0 and len(self.prev_ranges) == n_pts:
                vx, vy, vyaw = self._compute_motion(
                    self.prev_ranges,
                    ranges,
                    self.prev_valid,
                    valid,
                    angles,
                    dt,
                )

                # Integrate displacement in robot world frame
                dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
                dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
                self.x += dx
                self.y += dy
                self.yaw += vyaw * dt
                # Normalize yaw to [-pi, pi]
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

                self._publish_odometry(current_time, vx, vy, vyaw)

        self.prev_ranges = ranges.copy()
        self.prev_valid = valid.copy()
        self.prev_angle_min = msg.angle_min
        self.prev_angle_increment = msg.angle_increment
        self.last_time = current_time

    def _compute_motion(self, prev_ranges, curr_ranges, prev_valid, curr_valid, angles, dt):
        """Match laser beam returns using SVD Kabsch alignment."""
        # Find beams that struck valid obstacles in both consecutive scans
        common_valid = prev_valid & curr_valid

        # Reject large radial jumps indicating dynamic obstacles or occlusion
        dist_diff = np.abs(curr_ranges - prev_ranges)
        consistent = common_valid & (dist_diff < 0.35)

        num_matched = np.count_nonzero(consistent)
        if num_matched < 15:
            return 0.0, 0.0, 0.0

        p_ranges = prev_ranges[consistent]
        c_ranges = curr_ranges[consistent]
        matched_angles = angles[consistent]

        # Cartesian coordinates in sensor frame
        px = p_ranges * np.cos(matched_angles)
        py = p_ranges * np.sin(matched_angles)
        cx = c_ranges * np.cos(matched_angles)
        cy = c_ranges * np.sin(matched_angles)

        prev_pts = np.column_stack((px, py))
        curr_pts = np.column_stack((cx, cy))

        prev_centroid = np.mean(prev_pts, axis=0)
        curr_centroid = np.mean(curr_pts, axis=0)

        pv = prev_pts - prev_centroid
        cv = curr_pts - curr_centroid

        # Cross-covariance matrix
        H = pv.T @ cv

        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # Displacement from prev to curr
        translation = curr_centroid - R @ prev_centroid
        yaw_disp = math.atan2(R[1, 0], R[0, 0])

        # Velocities = displacement / dt
        vx = float(translation[0] / dt)
        vy = float(translation[1] / dt)
        vyaw = float(yaw_disp / dt)

        # Sanity velocity clamp for ground mobile robot
        vx = max(-1.0, min(1.0, vx))
        vy = max(-0.5, min(0.5, vy))
        vyaw = max(-2.0, min(2.0, vyaw))

        return vx, vy, vyaw

    def _publish_odometry(self, stamp, vx, vy, vyaw):
        q = self._yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose = Pose(position=Point(x=self.x, y=self.y, z=0.0), orientation=q)
        odom.twist.twist = Twist(
            linear=Vector3(x=vx, y=vy, z=0.0), angular=Vector3(x=0.0, y=0.0, z=vyaw)
        )

        # 6x6 covariance matrices (x, y, z, roll, pitch, yaw)
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.02   # x
        pose_cov[7] = 0.02   # y
        pose_cov[14] = 1e6   # z
        pose_cov[21] = 1e6   # roll
        pose_cov[28] = 1e6   # pitch
        pose_cov[35] = 0.05  # yaw
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.02  # vx
        twist_cov[7] = 0.02  # vy
        twist_cov[14] = 1e6  # vz
        twist_cov[21] = 1e6  # vroll
        twist_cov[28] = 1e6  # vpitch
        twist_cov[35] = 0.05 # vyaw
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        half = yaw * 0.5
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def main(args=None):
    rclpy.init(args=args)
    node = RF2oOdometryNode()
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
