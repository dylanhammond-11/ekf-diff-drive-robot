"""
Waypoint feedback controller
Subscribes to the EKF's
estimate (not ground truth!) and publishes cmd_vel.

"""
import math
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        # Waypoints as a flat parameter: [x0, y0, x1, y1, ...]
        default_waypoints = [0.0, 3.0, 2.0, 1.0, 4.0, 3.0, 4.0, 0.0]
        self.declare_parameter('waypoints_flat', default_waypoints)
        self.declare_parameter('kv', 1.0)
        self.declare_parameter('kw', 1.0)
        self.declare_parameter('rate_hz', 10.0)

        flat = self.get_parameter('waypoints_flat').value
        self.waypoints = np.array(flat).reshape(-1, 2)  # back to Nx2
        self.waypt_idx = 0

        self.kv = self.get_parameter('kv').value
        self.kw = self.get_parameter('kw').value

        #Latest state estimate, cached from the EKF
        self.latest_state = np.zeros(3)  # x, y, theta
        self.have_estimate = False

        # subscribe to ekf node
        self.estimate_sub = self.create_subscription(
            Odometry, 'ekf/estimate', self.estimate_cb, 10)

        # publish the (v, w) input commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # publish at the dt rate
        self.control_period = 1.0 / self.get_parameter('rate_hz').value
        self.control_timer = self.create_timer(self.control_period, self.control_cb)

        self.get_logger().info(
            f'Controller started with {len(self.waypoints)} waypoints')

    def estimate_cb(self, msg: Odometry):
        # get x, y, theta values from ekf node
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Inverse of the yaw-only quaternion formula
        # z = sin(theta/2), w = cos(theta/2)  =>  theta = 2 * atan2(z, w)
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)

        self.latest_state = np.array([x, y, theta])
        self.have_estimate = True

    def control_cb(self):
        if not self.have_estimate:
            return  # nothing to steer on yet

        v_cmd, w_cmd, self.waypt_idx = self.waypoint_controller(
            self.latest_state, self.waypoints, self.waypt_idx)

        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)

    
    def waypoint_controller(self, state, waypts, waypt_idx):
        num_of_waypoints = len(waypts)

        dx = waypts[waypt_idx, 0] - state[0]
        dy = waypts[waypt_idx, 1] - state[1]
        distance = math.sqrt(dx**2 + dy**2)

        theta_des = np.arctan2(dy, dx)
        dtheta = theta_des - state[2]
        dtheta_wrap = (dtheta + np.pi) % (2 * np.pi) - np.pi

        # HYSTERESIS FIX
        # Initialize hysteresis tracking flag if it doesn't exist
        if not hasattr(self, 'is_rotating'):
            self.is_rotating = False

        # Thresholds: Stop driving if error > 20 deg; keep rotating until < 5 deg
        if abs(dtheta_wrap) > np.radians(20.0):
            self.is_rotating = True
        elif abs(dtheta_wrap) < np.radians(5.0):
            self.is_rotating = False

        if self.is_rotating:
            v = 0.0
            w = self.kw * dtheta_wrap
        else:
            v = self.kv * distance
            w = self.kw * dtheta_wrap

        v_cmd_target = np.clip(v, 0.0, 1.5)
        w_cmd_target = np.clip(w, -1.5, 1.5)

    
        # Smooth out sudden velocity jumps 
        # Essentially an incremental controller

        max_accel = 2.0  # m/s^2 (Limit on max accel)
        max_alpha = 2.5 # rad/s^2
        max_v_step = max_accel * self.control_period
        max_w_step = max_alpha * self.control_period  

        if not hasattr(self, 'prev_v'):
            self.prev_v = 0.0
        
        dv = np.clip(v_cmd_target - self.prev_v, -max_v_step, max_v_step)
        v_cmd = self.prev_v + dv
        self.prev_v = v_cmd

        if not hasattr(self, 'prev_w'):
            self.prev_w = 0.0
    
        dw = np.clip(w_cmd_target - self.prev_w, -max_w_step, max_w_step)
        w_cmd = self.prev_w + dw
        self.prev_w = w_cmd

        # WAYPOINT ADVANCEMENT
        if distance < 0.08 and waypt_idx < num_of_waypoints - 1:
            waypt_idx += 1
            self.is_rotating = True  # Immediately pivot for the next point

        elif waypt_idx == num_of_waypoints - 1 and distance < 0.08:
            v_cmd = 0.0
            w_cmd = 0.0
            self.prev_v = 0.0
            self.prev_w = 0.0
            

        return v_cmd, w_cmd, waypt_idx


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()