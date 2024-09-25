#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('sliding_window_lane_detection'),
        'config',
        'config.yaml')

    decrypt_param_file = os.path.join(
        get_package_share_directory('decrypt_data'),
        'config',
        'config.yaml')

    ld = LaunchDescription()


    # Node #
    output_node=Node(
            package='sliding_window_lane_detection',
            executable='lane_detect_node',
            parameters = ['/home/avees/ros2_ws/src/sliding_window_lane_detection/config/config.yaml'],
            output='screen' 
            )
    
    decrypt_node=Node(
            package='decrypt_data',
            executable='decryption_node',
            parameters=['/home/avees/ros2_ws/src/decrypt_data/config/config.yaml'],
            output='screen')


    ld.add_action(output_node)
    ld.add_action(decrypt_node)

    return ld


