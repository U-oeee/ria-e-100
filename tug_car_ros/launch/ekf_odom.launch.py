from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('tug_car_ros')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_odom.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            output='screen',
            parameters=[ekf_config]
        )
    ])

