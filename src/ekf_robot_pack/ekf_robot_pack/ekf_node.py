'''
Node for EKF simulation

Subscribes to /gps, /imu, /encoder
Publishes through /ekf_pose to plot_node

'''

import numpy as np
import math
import rclpy
from rclpy.node import Node

# Adjust these to whatever msg types your gps/imu/encoder nodes actually publish.
from geometry_msgs.msg import TwistStamped, PointStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from ekf_robot_pack.ekf_core import EKF  # the pure-math class from ekf_core.py


class EKFNode(Node):
  
    def __init__(self):
        super().__init__('ekf_node')

        # Parameters for EKF
        self.declare_parameter('q_diag', [1e-3, 1e-3, 1e-3])
        self.declare_parameter('r_diag', [0.1, 0.1])
        self.declare_parameter('p0_diag', [0.05, 0.05, 0.05])
        self.declare_parameter('predict_rate_hz', 10.0)

        q = np.diag(self.get_parameter('q_diag').value)
        r = np.diag(self.get_parameter('r_diag').value)
        p0 = np.diag(self.get_parameter('p0_diag').value)
        h = np.array([[1, 0, 0], [0, 1, 0]])

        # EKF instance
        self.ekf = EKF(x=np.zeros(3), P=p0, Q=q, R=r, H=h)

        # Cached latest inputs (v, w), updated by subscriber callbacks
        self.latest_v = 0.0
        self.latest_omega = 0.0
        self.last_predict_time = self.get_clock().now()

        # Subscriptions
        # For encoder and IMU, just store upon callback. Prediction handled by timer
        self.encoder_sub = self.create_subscription(
            TwistStamped, '/enc_data', self.encoder_cb, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_cb, 10)
        # For GPS, update and publish upon callback
        self.gps_sub = self.create_subscription(
            PointStamped, '/gps', self.gps_cb, 10)

        # publish the estimate
        self.estimate_pub = self.create_publisher(Odometry, 'ekf/estimate', 10)

        # Predict step handeled by timer. Act on latest values of IMU and encoder measurements
        predict_period = 1.0 / self.get_parameter('predict_rate_hz').value
        self.predict_timer = self.create_timer(predict_period, self.predict_cb)

        self.get_logger().info('EKF node started')

    # Subscriber callbacks: latest values are stored
    def encoder_cb(self, msg: TwistStamped):
        self.latest_v = msg.twist.linear.x

    def imu_cb(self, msg: Imu):
        self.latest_omega = msg.angular_velocity.z

    def gps_cb(self, msg: PointStamped):
        z = np.array([msg.point.x, msg.point.y])
        self.ekf.update(z)
        self.publish_estimate()

    # Timer callback: predict step 
    def predict_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_predict_time).nanoseconds * 1e-9
        self.last_predict_time = now

        if dt <= 0.0:
            return  # guard against first call issues

        self.ekf.predict(v=self.latest_v, omega=self.latest_omega, dt=dt)
        self.publish_estimate()


    def publish_estimate(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = float(self.ekf.x[0])
        msg.pose.pose.position.y = float(self.ekf.x[1])
 
        # theta -> quaternion (yaw-only rotation, since this is a planar robot:
        theta = self.ekf.x[2]
        msg.pose.pose.orientation.x = 0.0  # x = 0 * sin(θ/2) = 0
        msg.pose.pose.orientation.y = 0.0  # y = 0 * sin(θ/2) = 0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
 
        # Diagonal covariance: Odometry's 6x6 covariance array (36 element array) (x, y, ... yaw)
        cov = [0.0] * 36
        cov[0] = self.ekf.P[0, 0]   # x-x
        cov[7] = self.ekf.P[1, 1]   # y-y
        cov[35] = self.ekf.P[2, 2]  # yaw-yaw
        msg.pose.covariance = cov
 
        self.estimate_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()