from setuptools import setup
import os
from glob import glob
package_name = 'tug_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/teleop.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'evdev'],
    zip_safe=True,
    maintainer='recs',
    maintainer_email='recs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_motor = tug_car.joy_motor_node:main',
        ],
    },
)
