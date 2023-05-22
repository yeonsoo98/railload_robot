import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, ThisLaunchFile
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    launch_dir  = os.path.join(get_package_share_directory('railload_robot_bringup'), 'launch')

    return LaunchDescription([

        # camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_dir  + '/camera.launch.py'),
        ),

        # delay
        TimerAction(
            period=10.0,  # delay in seconds
            actions=[
                # camera2
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_dir  + '/yolo.launch.py'),
                ),
            ],
        ),

        # # driving node
        # Node(
        #     package="railload_robot_bringup",
        #     executable="driving",
        #     name="driving",
        #     output="screen",
        # ),

        # # joystick commad node
        # Node(
        #     package="railload_robot_bringup",
        #     executable="joystick_command",
        #     name="joystick_command",
        #     output="screen",
        # ),

        # # keyboard commad node
        # Node(
        #     package="railload_robot_bringup",
        #     executable="keyboad_command",
        #     name="keyboard_command",
        #     output="screen",
        # ),

        # tracker node
        Node(
            package="railload_robot_bringup",
            executable="tracker",
            name="follower",
            output="screen",
        )

    ])

