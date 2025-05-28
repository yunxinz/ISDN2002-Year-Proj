import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class DynamicLidarTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_lidar_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        # Change topic name to your LiDAR pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/lidar_pose',  
            self.handle_pose,
            10
        )

    def handle_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'livox_frame' 
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicLidarTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
