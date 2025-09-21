from setuptools import setup
from glob import glob
import os

package_name = 'tug_car_ros'

def add_dir_if_has(pattern, install_subdir):
    files = glob(pattern)
    if files:
        return [(os.path.join('share', package_name, install_subdir), files)]
    return []

data_files_list = [
    # package index
    (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
     [os.path.join('resource', package_name)]),

    # package.xml
    (os.path.join('share', package_name), ['package.xml']),

    # launch files
    *add_dir_if_has('launch/*.launch.py', 'launch'),

    # ✅ support both config/ and params/ yaml locations
    *add_dir_if_has('config/*.yaml', 'config'),
    *add_dir_if_has('params/*.yaml', 'params'),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files_list,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='RPLIDAR + xIMU3 bringup (IMU publisher/odom/TF, SLAM ready)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_tf_publisher = tug_car_ros.imu_tf_publisher:main',
            'ximu3_publisher  = tug_car_ros.ximu3_publisher:main',
            'imu_odom_node    = tug_car_ros.imu_odom_node:main',
        ],
    },
)

