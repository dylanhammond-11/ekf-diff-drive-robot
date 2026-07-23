import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Define variables outside the list
    config_path = os.path.join(
        get_package_share_directory('ekf_robot_pack'),
        'config',
        'sim_params.yaml'
    )

    # 2. Pass only launch actions inside the LaunchDescription list
    return LaunchDescription([
        Node(
            package='ekf_robot_pack',
            executable='sim_node',
            name='sim_node',
            output='screen',
        ),
        Node(
            package='ekf_robot_pack',
            executable='gps_node',
            name='gps_node',
            output='screen',
        ),
        Node(
            package='ekf_robot_pack',
            executable='imu_node',
            name='imu_node',
            output='screen',
        ),
        Node(
            package='ekf_robot_pack',
            executable='encoder_node',
            name='encoder_node',
            output='screen',
        ),
        Node(
            package='ekf_robot_pack',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[config_path],
        ),
        Node(
            package='ekf_robot_pack',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[config_path],
        ),
    ])