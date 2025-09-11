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
            "launch", "display.launch.py"
        ),
    )
    
    # Launch gazebo - not used for real robot
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_description"),
            "launch", "gazebo.launch.py"
        ),
    )
    
    # Launch hardware - mandatory for real robot
    hardware = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_firmware"),
            "launch", "hardware.launch.py"
        ),
    )
    
    # Launch the controller manager spawner
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rmitbot_controller"),
            "launch", "controller.launch.py"
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
            "launch", "teleopkeyboard.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
    
    # If the ESP32 is connected to the PC, then launch from the PC
    # display
    # hardware, 
    # controller_delayed
    # teleopkeyboard
    
    # If the ESP32 is connected to the RPI, then launch from the PC
    # display
    # teleopkeyboard
    # and launch from the RPI
    # hardware, 
    # controller_delayed
    
    return LaunchDescription([
        display,                # Must be launched by the PC
        # hardware,             # if ESP32 is connected to the PC via USB, launch from PC, otherwise launch from RPI
        # gazebo,               # not used for real robot
        # controller_delayed,   # if ESP32 is connected to the PC via USB, launch from PC, otherwise launch from RPI
        teleopkeyboard,         # Must be launched by the PC
    ])