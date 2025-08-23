from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(":")[0],
        'share',
        'e100_description',
        'urdf',
        'ria_e100.urdf.xacro'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': os.popen(f'xacro {urdf_path}').read()
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                os.getenv('AMENT_PREFIX_PATH').split(":")[0],
                'share',
                'e100_description',
                'launch',
                'view.launch.rviz'  # RViz 설정 파일 경로
            )]
        )
    ])

