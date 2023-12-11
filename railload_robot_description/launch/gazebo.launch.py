import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = True

    world_path = PathJoinSubstitution(
        [FindPackageShare("railload_robot_description"), "worlds", "test.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('railload_robot_description'), 'launch', 'display.launch.py']
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "railload_robot",
                        "-x", "0.0", "-y", "0.0", "-z", "0.0"]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
            }.items()
        ),

    ])