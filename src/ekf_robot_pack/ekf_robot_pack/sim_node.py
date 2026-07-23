'''
Node for the true state diff drive robot simulation

Publishes through /true_pose to imu_node, ekf_node, and gps_node

Subscribes to the controller node which publishes the robot velocity commands: v_cmd and w_cmd

'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
import math


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('sim_node')

        # Init state and commands
        self.state_true = np.array([0,0,0])
        self.v_cmd = 0.0 # init to 0
        self.w_cmd = 0.0 # init to 0
        self.dt = 0.1

        # Publisher for the true robot pose
        self.state_pub = self.create_publisher(Odometry,'/true_pose',10)

        # Subsciber to the controller which will give v_cmd and w_cmd
        self.cmd_sub = self.create_subscription(Twist,'/cmd_vel', self.cmd_vel_callback, 10 )

        # Timer for publishing state
        self.timer = self.create_timer(self.dt, self.update_state)

       


    def update_state(self):

        # Update true state callback
        self.state_true = diff_drive_model(self.state_true, self.v_cmd, self.w_cmd, self.dt)

        # Publish
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = self.state_true[0]
        msg.pose.pose.position.y = self.state_true[1]
        qz, qw = theta_to_quaternion(self.state_true[2])
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.v_cmd
        msg.twist.twist.angular.z = self.w_cmd
        # Publish
        self.state_pub.publish(msg)
        # Log
        self.get_logger().info(f'true state: {self.state_true}')
    
    def cmd_vel_callback(self, msg):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z
    





def diff_drive_model(state, v_inp:float, w_inp:float, dt:float):
    '''

    state for differential drive robot: [ x, y, theta]
    inputs are v_inp and w_inp

    '''
    x = state[0]
    y = state[1]
    theta = state[2]

    xdot = v_inp * math.cos(theta)
    ydot = v_inp * math.sin(theta)
    thetadot = w_inp

    x_new = x + xdot*dt
    y_new = y + ydot*dt
    theta_new = theta + thetadot*dt

    # Wrap the angle from -pi to pi
    theta_new = (theta_new + np.pi) % (2*np.pi) - np.pi

    state = np.array([x_new, y_new, theta_new])

    return state
    
def theta_to_quaternion(theta):
    # Pure 2D (yaw-only) rotation - x and y components are always 0
    return math.sin(theta / 2.0), math.cos(theta / 2.0)  # qz, qw


        

def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()



            
