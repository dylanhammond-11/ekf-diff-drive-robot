# EKF Differential Drive Robot

A ROS2 (simulation-based) differential drive robot demonstrating Extended Kalman Filter
state estimation.

## Overview

Simulates a differential drive robot navigating between waypoints, fusing noisy
GPS, IMU, and wheel encoder measurements through an EKF to estimate pose (x, y, theta)
in real time, and comparing the estimate against ground truth.
EKF is designed from scratch (see `ekf_core.py`): prediction step, measurement update, and Jacobians

## Architecture
- **sim_node** — simulates true robot state given velocity commands (diff-drive kinematics)
- **gps_node / imu_node / encoder_node** — simulate noisy sensor measurements from true state
- **ekf_node** — fuses encoder/IMU (prediction) and GPS (correction) into a pose estimate
- **controller_node** — drives the robot through a sequence of waypoints using the EKF estimate
- **plot_node** — live matplotlib visualization of true vs. estimated trajectory, with covariance ellipse

## Running it

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ekf_robot_pack ekf_sim_launch.py
```

To view the live trajectory plot, open a second terminal, source the workspace again, and run:

```bash
source install/setup.bash
ros2 run ekf_robot_pack plot_node
```

## Configuration

Tunable parameters (waypoints, controller gains, EKF noise covariances) live in
`config/sim_params.yaml` 

## Status

Complete: full pipeline (simulated sensors, EKF fusion, waypoint controller, live trajectory plotting) working end-to-end in simulation. Still working to refine and add new features.