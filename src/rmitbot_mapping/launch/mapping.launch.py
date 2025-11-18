import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

# ros2 launch rmitbot_mapping slam.launch.py use_sim_time:=true
# ros2 launch rmitbot_mapping slam.launch.py use_sim_time:=false
# Command line
# ros2 launch slam_toolbox online_async_launch.py params_file:=./scr/rmitbot_mapping/config/slam.yaml use_sim_time:=true

def generate_launch_description():
    
    slam_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("slam_toolbox"),"launch","online_async_launch.py"),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory("rmitbot_mapping"), "config", "slam.yaml"),             
            'use_sim_time': "False", 
            }.items()
    )
    
    return LaunchDescription([
        slam_launch,
    ])
    