import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

# Launch the file
# ros2 launch rmitbot_bringup rmitbot.launch.py

def generate_launch_description():
    
    # Launch rviz
    display = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_description"),
            "launch",
            "display.launch.py"
        ),
    )
    
    # Launch gazebo
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    # Launch the controller manager
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )
    
    # Launch the controller manager 3s after gazebo, to make sure the robot has spawned in simulation
    controller_delayed = TimerAction(
        period = 3., 
        actions=[controller]
    )
    
    # Launch the teleop keyboard node
    teleopkeyboard = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_controller"),
            "launch",
            "teleopkeyboard.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )
    
    return LaunchDescription([
        display, 
        gazebo,
        controller_delayed, 
        teleopkeyboard,
    ])