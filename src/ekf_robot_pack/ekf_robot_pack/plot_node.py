#!/usr/bin/env python3
"""
Live plotter node for the EKF diff-drive robot project.

Subscribes to:
  /true_pose      (nav_msgs/Odometry): ground truth from sim_node
  /ekf/estimate   (nav_msgs/Odometry): EKF state estimate + covariance
  /cmd_vel        (geometry_msgs/Twist): controller_node's velocity commands

Parameters:
  waypoints_flat  (double_array): Flattened array of waypoint coordinates [x0, y0, x1, y1, ...]
  waypoint_radius (double): Target acceptance radius (m)

Plots (live, updates as data arrives):
  - Left:         True vs. estimated XY trajectory with waypoint dots dynamically
                  loaded from ROS parameters, 85%-confidence ellipses, and dynamic
                  heading triangles representing current robot orientation.
  - Top-right:    1-sigma covariance bounds vs. time.
  - Bottom-right: Velocity commands (v_cmd, w_cmd) vs. time.
"""

import threading
import math
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Ellipse, Polygon
from matplotlib.animation import FuncAnimation

from scipy.stats import chi2

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


HISTORY_LEN = 2000
CONFIDENCE = 0.85
ELLIPSE_STRIDE = 20
ROBOT_TRIANGLE_SIZE = 0.04  # Meters 


def yaw_from_quaternion(q):
    """
    Extracts yaw (rotation around Z axis) from a geometry_msgs/Quaternion.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def get_triangle_vertices(x, y, yaw, size=ROBOT_TRIANGLE_SIZE):
    """
    Computes transformed 2D triangle vertices given pose (x, y, yaw).
    """
    pts = np.array([
        [size, 0.0],           # Nose
        [-size / 2, size / 2], # Left rear
        [-size / 2, -size / 2] # Right rear
    ])
    
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    
    rotated = (R @ pts.T).T
    return rotated + np.array([x, y])


def cov_ellipse_params(P_xy, confidence):
    """
    Given a 2x2 position covariance block, return a confidence ellipse (angle_deg, width, height)
    for a matplotlib Ellipse patch.
    """
    vals, vecs = np.linalg.eigh(P_xy)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    angle = math.degrees(math.atan2(vecs[1, 0], vecs[0, 0]))
    s = np.sqrt(chi2.ppf(confidence, df=2))

    width, height = 2 * s * np.sqrt(np.clip(vals, 0, None))

    return angle, width, height


def add_waypoint_overlay(ax, waypoints, radius):
    """
    Helper function to render nominal waypoint dots and switching 
    tolerance circles onto a Matplotlib axis.
    """
    if len(waypoints) == 0:
        return

    # 1. Add waypoint target dots
    ax.plot(
        waypoints[:, 0], waypoints[:, 1],
        'ro', markersize=13, alpha=0.5, zorder=2, label='Waypoints'
    )


class PlotNode(Node):

    def __init__(self):
        super().__init__(
            'plot_node',
            automatically_declare_parameters_from_overrides=True
        )
        self.lock = threading.Lock()

        # Declare ROS 2 parameters
        if not self.has_parameter('waypoints_flat'):
            self.declare_parameter(
                'waypoints_flat',
               [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 2.0, 1.0, 2.0]
               #[0.0,0.0, 10.0,10.0, 20.0, 20.0]
            )
        if not self.has_parameter('waypoint_radius'):
            self.declare_parameter('waypoint_radius', 0.18)

        # Parse waypoints into Nx2 matrix
        flat_wp = self.get_parameter('waypoints_flat').get_parameter_value().double_array_value
        if len(flat_wp) > 0 and len(flat_wp) % 2 == 0:
            self.waypoints = np.array(flat_wp).reshape(-1, 2)
        else:
            self.waypoints = np.empty((0, 2))

        self.waypoint_radius = self.get_parameter('waypoint_radius').get_parameter_value().double_value

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.true_xy = deque(maxlen=HISTORY_LEN)
        self.true_yaw = deque(maxlen=HISTORY_LEN)
        self.est_xy = deque(maxlen=HISTORY_LEN)
        self.est_yaw = deque(maxlen=HISTORY_LEN)
        self.est_t = deque(maxlen=HISTORY_LEN)
        self.est_std = deque(maxlen=HISTORY_LEN)
        self.latest_P_xy = np.eye(2) * 1e-6
        self.est_count = 0

        self.cmd_t = deque(maxlen=HISTORY_LEN)
        self.cmd_v = deque(maxlen=HISTORY_LEN)
        self.cmd_w = deque(maxlen=HISTORY_LEN)

        self.create_subscription(Odometry, '/true_pose', self.cb_true, 10)
        self.create_subscription(Odometry, '/ekf/estimate', self.cb_ekf, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)

    def cb_true(self, msg: Odometry):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self.lock:
            self.true_xy.append((msg.pose.pose.position.x,
                                 msg.pose.pose.position.y))
            self.true_yaw.append(yaw)

    def cb_ekf(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        cov = msg.pose.covariance

        P_xy = np.array([[cov[0], cov[1]],
                         [cov[6], cov[7]]])

        sigma_x = math.sqrt(max(cov[0], 0.0))
        sigma_y = math.sqrt(max(cov[7], 0.0))
        sigma_yaw = math.sqrt(max(cov[35], 0.0))

        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0

        with self.lock:
            self.est_xy.append((x, y))
            self.est_yaw.append(yaw)
            self.est_t.append(t)
            self.est_std.append((sigma_x, sigma_y, sigma_yaw))
            self.latest_P_xy = P_xy
            self.est_count += 1

    def cb_cmd(self, msg: Twist):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0

        with self.lock:
            self.cmd_t.append(t)
            self.cmd_v.append(msg.linear.x)
            self.cmd_w.append(msg.angular.z)


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = PlotNode()

    spin_thread = threading.Thread(
        target=ros_spin_thread,
        args=(node,),
        daemon=True
    )
    spin_thread.start()

    fig = plt.figure(figsize=(14, 6))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.2, 1])

    ax_traj = fig.add_subplot(gs[:, 0])
    ax_cov = fig.add_subplot(gs[0, 1])
    ax_cmd = fig.add_subplot(gs[1, 1], sharex=ax_cov)

    # --- Trajectory Axis Configuration ---
    ax_traj.set_title('Robot Trajectory: True vs EKF Estimate')
    ax_traj.set_xlabel('x (m)')
    ax_traj.set_ylabel('y (m)')
    ax_traj.axis('equal')

    # Dynamically draw waypoint dots from ROS parameter server
    add_waypoint_overlay(ax_traj, node.waypoints, node.waypoint_radius)

    true_line, = ax_traj.plot([], [], 'b-',linewidth=2.2, zorder=3, label='True State')
    est_line, = ax_traj.plot([], [], 'g-', linewidth=2.0,zorder=4, label='EKF Estimate')

    # Robot Heading Indicator Triangles
    true_triangle = Polygon([[0, 0], [0, 0], [0, 0]], facecolor='blue', edgecolor='darkblue', alpha=0.9, zorder=6)
    est_triangle = Polygon([[0, 0], [0, 0], [0, 0]], facecolor='limegreen', edgecolor='darkgreen', alpha=0.9, zorder=7)
    
    ax_traj.add_patch(true_triangle)
    ax_traj.add_patch(est_triangle)

    live_ellipse = Ellipse(
        (0, 0), 0, 0,
        edgecolor='g', facecolor='none', linestyle='--', alpha=0.6, zorder=5,
        label='EKF Covariance (85%)'
    )
    ax_traj.add_patch(live_ellipse)
    
    ax_traj.legend(
    loc='upper left', 
    bbox_to_anchor=(1.02, 1.0), 
    borderaxespad=0,
    framealpha=0.9
)

    # --- Covariance Time-Series Axis ---
    ax_cov.set_title('EKF 1-sigma Bounds vs Time')
    ax_cov.set_ylabel('sigma (m or rad)')

    sx_line, = ax_cov.plot([], [], label='sigma_x (m)')
    sy_line, = ax_cov.plot([], [], label='sigma_y (m)')
    syaw_line, = ax_cov.plot([], [], label='sigma_yaw (rad)')

    ax_cov.legend(loc='upper right')
    ax_cov.tick_params(labelbottom=False)

    # --- Command Time-Series Axis ---
    ax_cmd.set_title('Control Inputs')
    ax_cmd.set_xlabel('Time (s)')
    ax_cmd.set_ylabel('command')

    v_line, = ax_cmd.plot([], [], label='v_cmd (Linear)')
    w_line, = ax_cmd.plot([], [], label='w_cmd (Angular)')

    ax_cmd.legend(loc='upper right')

    def update(_frame):
        with node.lock:
            true_xy = list(node.true_xy)
            true_yaw = list(node.true_yaw)
            est_xy = list(node.est_xy)
            est_yaw = list(node.est_yaw)
            est_t = list(node.est_t)
            est_std = list(node.est_std)
            P_xy = node.latest_P_xy.copy()
            count = node.est_count
            cmd_t = list(node.cmd_t)
            cmd_v = list(node.cmd_v)
            cmd_w = list(node.cmd_w)

        if true_xy:
            tx, ty = zip(*true_xy)
            true_line.set_data(tx, ty)

            # Update true heading triangle patch
            true_verts = get_triangle_vertices(tx[-1], ty[-1], true_yaw[-1])
            true_triangle.set_xy(true_verts)

        if est_xy:
            ex, ey = zip(*est_xy)
            est_line.set_data(ex, ey)

            # Update EKF heading triangle patch
            est_verts = get_triangle_vertices(ex[-1], ey[-1], est_yaw[-1])
            est_triangle.set_xy(est_verts)

            angle, width, height = cov_ellipse_params(P_xy, CONFIDENCE)

            live_ellipse.set_center((ex[-1], ey[-1]))
            live_ellipse.angle = angle
            live_ellipse.width = width
            live_ellipse.height = height

            if count % ELLIPSE_STRIDE == 0 and count > 0:
                patch = Ellipse(
                    (ex[-1], ey[-1]), width, height, angle=angle,
                    edgecolor='gray', facecolor='none', alpha=0.3, zorder=2
                )
                ax_traj.add_patch(patch)

            ax_traj.relim()
            ax_traj.autoscale_view()

        if est_t:
            sx, sy, syaw = zip(*est_std)
            sx_line.set_data(est_t, sx)
            sy_line.set_data(est_t, sy)
            syaw_line.set_data(est_t, syaw)

            ax_cov.relim()
            ax_cov.autoscale_view()

        if cmd_t:
            v_line.set_data(cmd_t, cmd_v)
            w_line.set_data(cmd_t, cmd_w)

            ax_cmd.relim()
            ax_cmd.autoscale_view()

        return (
            true_line, est_line, live_ellipse, true_triangle, est_triangle,
            sx_line, sy_line, syaw_line, v_line, w_line
        )

    ani = FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)

    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()