import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Launch the file
# ros2 launch rmitbot_description gazebo.launch.py

def generate_launch_description():
    # Path to the package
    pkg_path_description = get_package_share_directory("rmitbot_description")
    
    # Resource path for gazebo. Required while using stl (robot CAD), and sdf (world)
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_path_description).parent.resolve())]
    )

    # Launch Gazebo 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf", " --render-engine", " ogre"])]
    )
    
    # Spawn the robot in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description","-name", "rmitbot"],
    )

    # Bridge between ROS2 and Gazebo
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], 
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])