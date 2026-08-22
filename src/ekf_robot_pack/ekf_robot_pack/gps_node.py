'''
Node for Simulated GPS

Subscribes to /true_pose
Publishes to ekf_node

'''

import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import numpy as np


class GPSNode(Node):

    def __init__(self):

        super().__init__('gps_node')

        self.declare_parameter('gps_noise_std', 0.15)
        self.gps_noise_std = self.get_parameter('gps_noise_std').value

        # Declare GPS sampling frequency as a parameter
        self.declare_parameter('gps_hz', 5.0)
        self.gps_hz = self.get_parameter('gps_hz').value
        self.gps_period = 1/self.gps_hz

        # Publish: gps pose
        self.gps_pub = self.create_publisher(PointStamped, '/gps', 10)

        # Subscribe: /true_pose
        self.true_pose_sub = self.create_subscription(Odometry,'/true_pose', self.true_pose_callback, 10)


        # Create publish timer. Publish GPS at established frequency
        self.timer = self.create_timer(self.gps_period, self.gps_pub_callback)


        # Robot state
        self.x = 0.0
        self.y = 0.0


    def true_pose_callback(self, msg):
        # Pull out pose (2d or quaterion?)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        

    def gps_pub_callback(self):
        # GPS message
        msg_gps = PointStamped()
        msg_gps.header.stamp = self.get_clock().now().to_msg()
        msg_gps.header.frame_id = 'gps_link'
        # Add gaussian noise
        msg_gps.point.x = self.x + np.random.normal(0, self.gps_noise_std)
        msg_gps.point.y = self.y + np.random.normal(0, self.gps_noise_std) 
        # publish
        self.gps_pub.publish(msg_gps)


def main(args=None):

    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()