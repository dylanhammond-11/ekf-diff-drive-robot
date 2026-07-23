'''
Node for the simulated IMU sensor

Subscribes to /true_pose, adds noise to the true angular velocity,
publishes the noisy reading on /imu/data
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import numpy as np


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Publish IMU Data
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        # Subscribe to the true pose from sim_node.
        self.true_pose_sub = self.create_subscription(Odometry,'/true_pose', self.true_pose_callback, 10)

    def true_pose_callback(self, msg):
        # Pull out angluar velocity
        w_true = msg.twist.twist.angular.z
        # Add gaussian noise
        w_imu = w_true + np.random.normal(0,0.1)

        # IMU message
        msg_imu = Imu()
        msg_imu.header.stamp = self.get_clock().now().to_msg()
        msg_imu.header.frame_id = 'imu_link'
        msg_imu.angular_velocity.z = w_imu
        # publish
        self.imu_pub.publish(msg_imu)


        pass


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()