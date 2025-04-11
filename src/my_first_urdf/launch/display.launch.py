from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

# This file is used to launch the robot in RViz2
# It will load the 
# - URDF file, 
# - robot_state_publisher, 
# - joint_state_publisher_gui
# - display it in RViz2

def generate_launch_description():
    # <pkg_name>_dir = get_package_share_directory("<pkg_name>")
    my_first_urdf_dir = get_package_share_directory("my_first_urdf")
    
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(my_first_urdf_dir, 'urdf', 'my_robot.urdf.xacro'),
        description="Absolute path to robot urdf file"
    )
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description}]
        )
    
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(my_first_urdf_dir, 'rviz', 'display.rviz')],
    )
    
    
    return LaunchDescription([
        model_arg, 
        robot_state_publisher, 
        joint_state_publisher,
        rviz_node
    ])