import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }],
        )
    ])
