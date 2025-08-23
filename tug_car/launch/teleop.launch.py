from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            node_executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}]   # 필요 시 옵션
        ),
        Node(
            package='tug_car',
            node_executable='joy_motor',
            name='motor_control_node',
            output='screen'
        ),
    ])

