import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ros2 launch rmitbot_controller controller.launch.py

def generate_launch_description():
    
    # Path to the controller config file
    pkg_path = get_package_share_directory("rmitbot_controller")
    robot_controllers = os.path.join(pkg_path, 
                             'config', 
                             'rmitbot_controller.yaml')

    # joint_state_broadcaster (jsb): dynamic TF of the motor joints 
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    # controller: IK from Cartesian speed to motor speed command
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'mecanum_drive_controller',
            '--param-file',
            robot_controllers,
            '--controller-ros-args',
            '-r /mecanum_drive_controller/tf_odometry:=/tf',
            '--controller-ros-args',
            '-r /mecanum_drive_controller/reference:=/rmitbot_controller/cmd_vel',
        ],
    )
    
    # controller must be spawned after the jsb
    controller_spawner_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[controller_spawner],
            )
        )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            controller_spawner_after_jsb,
        ]
    )