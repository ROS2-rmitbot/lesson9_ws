import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Path to the package
    pkg_path_description = get_package_share_directory("rmitbot_description")
    
    pkg_path_controller = get_package_share_directory("rmitbot_controller")
    
    # Path to the urdf file
    urdf_path = os.path.join(pkg_path_description, 'urdf', 'rmitbot.urdf.xacro')
    
    # Path to the controller yaml file
    ctrl_yaml_path = os.path.join(pkg_path_controller, 'config', 'rmitbot_controller.yaml')
    
    # Compile the xacro file to urdf
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # robot_description = ParameterValue(
    #     Command(
    #         [
    #             "xacro ",
    #             os.path.join(
    #                 get_package_share_directory("rmitbot_description"),
    #                 "urdf",
    #                 "rmitbot.urdf.xacro",
    #             ),
    #             " is_sim:=False"
    #         ]
    #     ),
    #     value_type=str,
    # )

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{"robot_description": robot_description}],
    # )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            ctrl_yaml_path, 
        ],
    )

    return LaunchDescription(
        [
            # robot_state_publisher_node,
            controller_manager,
        ]
    )