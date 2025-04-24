import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# This file is used to launch the robot in Gazebo
# It will load the 
# - URDF file, 
# - robot_state_publisher, 
# - bunch of package from gazebo-gz

def generate_launch_description():
    # Get the directory of the package
    rmitbot_description_dir = get_package_share_directory("rmitbot_description")
    
    # Declare the model argument
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(rmitbot_description_dir, "urdf", "rmitbot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )
    
    # This line processes your robot’s .xacro file at launch time, converting it to URDF
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    # This node publishes the robot state to the TF tree
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )
    
    # This node sets the environment variable for Gazebo resource path
    # This line makes sure Gazebo can find your robot model
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(rmitbot_description_dir).parent.resolve())]
    )
    
    # This node launches Gazebo with the specified configuration file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf", " --render-engine", " ogre"])]
    )
    
    # This node spawns the robot in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description","-name", "rmitbot"],
    )

    # This node bridges the clock messages between ROS2 and Gazebo
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], 
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])