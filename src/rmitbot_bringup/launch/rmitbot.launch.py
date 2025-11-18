import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

# Launch the file
# ros2 launch rmitbot_bringup rmitbot.launch.py

def generate_launch_description():
    
    # Path to the package 
    pkg_path_description =  get_package_share_directory("rmitbot_description")
    pkg_path_controller =   get_package_share_directory("rmitbot_controller")
    pkg_path_localization =   get_package_share_directory("rmitbot_localization")
    pkg_path_mapping =   get_package_share_directory("rmitbot_mapping")
    
    # Launch rviz
    display = IncludeLaunchDescription(
        os.path.join(pkg_path_description,"launch","display.launch.py"),
    )  

    # Launch the controller manager
    controller = IncludeLaunchDescription(
        os.path.join(pkg_path_controller,"launch","controller.launch.py"),
    )
    
    
    # Launch the teleop keyboard node
    teleopkeyboard = IncludeLaunchDescription(
        os.path.join(pkg_path_controller,"launch", "teleopkeyboard.launch.py"),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
    
    localization = IncludeLaunchDescription(
        os.path.join(pkg_path_localization,"launch","localization.launch.py"),
    )    
    
    # Launch the rplidar hardware
    rplidar = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping,"launch", "rplidar.launch.py"),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
    
    # Launch the slamtoolbox 
    slamtoolbox = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping,"launch", "mapping.launch.py"),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
    
    # PC:   display, teleopkeyboard
    # RPI:  controller, localization, rplidar, slamtoolbox
    return LaunchDescription([
        display,
        # controller,
        teleopkeyboard,
        # localization,
        # rplidar, 
        # slamtoolbox, 
    ])