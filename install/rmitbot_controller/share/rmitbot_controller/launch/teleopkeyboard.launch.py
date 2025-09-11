import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# CLI command (just in case)
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/my_robot_controller/cmd_vel -p stamped:=True

def generate_launch_description():
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[
            {"use_sim_time": False},
            {'stamped': True}], 
        remappings=[
            ('cmd_vel', '/rmitbot_controller/cmd_vel')]
    )         
    
    return LaunchDescription(
        [
            teleop_keyboard, 
        ]
    )
    
    