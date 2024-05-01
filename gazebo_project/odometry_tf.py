import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TfPublisherNode(Node):

    def __init__(self):
        super().__init__("tf_publisher_node")
        self.pose_subscription = self.create_subscription(
            PoseArray, "/pose_info", self.pose_callback, 10  # Adjust QoS as needed
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Tf publisher node started.")

    def pose_callback(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().warn(
                "PoseArray message contains less than 2 poses. Skipping TF publishing."
            )
            return

        # Extract the desired pose (2nd pose)
        pose = msg.poses[1]

        # Create a TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation.w = pose.orientation.w
        transform.transform.rotation.x = pose.orientation.x
        transform.transform.rotation.y = pose.orientation.y
        transform.transform.rotation.z = pose.orientation.z

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug("Published TF from world to base_link.")


def main():
    rclpy.init()
    node = TfPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
