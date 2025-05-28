#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomFrameRemap(Node):
    def __init__(self):
        super().__init__('odometry_frame_remap')
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.odom_callback, 10)
        self.pub = self.create_publisher(
            Odometry, '/Odometry_fixed', 10)

    def odom_callback(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFrameRemap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
