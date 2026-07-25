import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__("static_tf_broadcaster")
        broadcaster = StaticTransformBroadcaster(self)

        laser_tf = TransformStamped()
        laser_tf.header.frame_id = "base_link"
        laser_tf.child_frame_id = "laser"
        laser_tf.transform.translation.x = 0.10
        laser_tf.transform.translation.y = 0.0
        laser_tf.transform.translation.z = 0.05
        laser_tf.transform.rotation.w = 1.0

        imu_tf = TransformStamped()
        imu_tf.header.frame_id = "base_link"
        imu_tf.child_frame_id = "imu_link"
        imu_tf.transform.translation.x = 0.05
        imu_tf.transform.translation.y = 0.0
        imu_tf.transform.translation.z = 0.0
        imu_tf.transform.rotation.w = 1.0

        broadcaster.sendTransform([laser_tf, imu_tf])
        self.get_logger().info(
            "Published static transforms: base_link -> laser, base_link -> imu_link"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
