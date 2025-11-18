import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

# Launch the file
# ros2 launch rmitbot_description display.launch.py

def generate_launch_description():
    # Path to the package
    pkg_path_description = get_package_share_directory("rmitbot_description")
    # Path to the rviz config file
    rviz_path = os.path.join(pkg_path_description, 'rviz', 'display.rviz')
    # This node launches RViz2 with the specified configuration file
    rviz = Node(
        package=    'rviz2',
        executable= 'rviz2',
        name=       'rviz2',
        output=     'screen',
        arguments=[ '-d', rviz_path],
        parameters=[{"use_sim_time": False}],
    )
    
    return LaunchDescription([
        rviz, 
    ])