import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

# This file is used to launch the robot in RViz2
# It will load the 
# - URDF file, for robot_state_publisher
# - robot_state_publisher: publish TF tree, for rviz
# - joint_state_publisher_gui: publish joint_state, for robot_state_publisher
# - RViz2, needs TF from robot_state_publisher and joint_state from joint_state_publisher_gui

def generate_launch_description():
    # Get the directory of the package
    rmitbot_description_dir = get_package_share_directory("rmitbot_description")
    
    # Declare the model argument
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(rmitbot_description_dir, 'urdf', 'rmitbot.urdf.xacro'),
        description="Absolute path to robot urdf file"
    )
    
    # This line processes your robot’s .xacro file at launch time, converting it to URDF
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    
    # This node publishes the robot state to the TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description}]
        )
    
    # This node publishes the joint state to the TF tree
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    
    # This node launches RViz2 with the specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(rmitbot_description_dir, 'rviz', 'display.rviz')],
    )
    
    return LaunchDescription([
        model_arg, 
        robot_state_publisher, 
        joint_state_publisher,
        rviz_node
    ])