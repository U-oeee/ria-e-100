from setuptools import setup

package_name = 'odom_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uoeee',
    maintainer_email='uoeee@todo.todo',
    description='Odometry fusion node (encoder + IMU)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = odom_ros2.odom:main',  # ✅ odom.py의 main 함수
        ],
    },
)

