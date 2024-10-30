from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)

def generate_launch_description():
    declared_arguments = []
    ld =LaunchDescription()

    arm_world_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm_gazebo'), "launch", "arm_world.launch.py")
        )
    )

    ld.add_action(arm_world_launch_file)

    arm_control_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm_control'), "launch", "arm_control.launch.py")
        )
    )

#    arm_control_launch_file = TimerAction(
#        period=10.0,
#        actions=[
#            IncludeLaunchDescription(
#                PythonLaunchDescriptionSource(
#                    os.path.join(get_package_share_directory('arm_control'), "launch", "arm_control.launch.py")
#                )
#            )
#        ]
#    )

    ld.add_action(arm_control_launch_file)


    return ld
