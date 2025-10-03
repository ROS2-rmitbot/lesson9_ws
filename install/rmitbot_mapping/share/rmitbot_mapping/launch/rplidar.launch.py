import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                # 'serial_port': '/dev/ttyUSB1',
                'serial_port': '/dev/rplidar',
                'frame_id': 'laser_link',
                'angle_compensate': True,
                'scan_mode': 'Standard', 
                'use_sim_time': False, # This is important for simulation/hardware
            }]
        )
    ])