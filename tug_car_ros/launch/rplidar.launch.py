from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='front_laser_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    dtr_control = LaunchConfiguration('dtr_control', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        DeclareLaunchArgument('inverted', default_value=inverted),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode),
        DeclareLaunchArgument('dtr_control', default_value=dtr_control),

        Node(
            package='sllidar_ros2',        # ← 여기!
            executable='sllidar_node',     # ← 보통 이 이름 (확인은 아래 3단계)
            name='rplidar',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
                'dtr_control': dtr_control
            }],
            output='screen'
        )
    ])

