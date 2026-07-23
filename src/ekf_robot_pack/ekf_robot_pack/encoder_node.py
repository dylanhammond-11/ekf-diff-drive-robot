
'''
Node for Simulated encoder

Subscribes to /true_state
Publishes to ekf_node

'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

import numpy as np


class ENCNode(Node):
    def __init__(self):
        super().__init__('enc_node')

        # Publish IMU Data
        self.enc_pub = self.create_publisher(TwistStamped, '/enc_data', 10)

        # Subscribe to the true pose from sim_node.
        self.true_pose_sub = self.create_subscription(Odometry,'/true_pose', self.true_pose_callback, 10)

    def true_pose_callback(self, msg):
        # Pull out angluar velocity
        v_true = msg.twist.twist.linear.x
        # Add gaussian noise
        v_imu = v_true + np.random.normal(0,0.1)

        # Encoder message
        msg_enc = TwistStamped()
        msg_enc.header.stamp = self.get_clock().now().to_msg()
        msg_enc.header.frame_id = 'enc_link'
        msg_enc.twist.linear.x = v_imu
        # publish
        self.enc_pub.publish(msg_enc)



def main(args=None):
    rclpy.init(args=args)
    node = ENCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()