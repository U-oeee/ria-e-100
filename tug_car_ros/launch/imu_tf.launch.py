from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tug_car_ros',
            executable='imu_tf_broadcaster',
            name='imu_tf_broadcaster',
            parameters=[
                {'parent_frame': 'map'},       # RViz Fixed Frame을 'map'으로 설정
                {'child_frame': 'imu_link'},   # 센서 프레임 이름
                {'imu_topic': '/imu/data'},    # x-IMU3 노드가 퍼블리시하는 토픽명
            ],
            output='screen'
        ),
        # 필요 시 RViz 자동 실행
        # Node(package='rviz2', executable='rviz2', name='rviz2'),
    ])

