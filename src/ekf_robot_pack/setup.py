from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ekf_robot_pack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dylanhammond',
    maintainer_email='dylanham011@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'ekf_node = ekf_robot_pack.ekf_node:main',
        'sim_node = ekf_robot_pack.sim_node:main',
        'gps_node = ekf_robot_pack.gps_node:main',
        'imu_node = ekf_robot_pack.imu_node:main',
        'encoder_node = ekf_robot_pack.encoder_node:main',
        'controller_node = ekf_robot_pack.controller_node:main',
        ],
    },
)
