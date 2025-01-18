#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get the package directory
    pkg_share = get_package_share_directory('robot_gazebo')
    
    # Paths to URDF and params
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    # Make sure the URDF exists
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f'URDF file not found: {urdf_file}')

    # Read the URDF into a string
    try:
        with open(urdf_file, 'r') as file:
            robot_description = file.read()
    except Exception as e:
        raise RuntimeError(f'Failed to read URDF: {str(e)}')

    return LaunchDescription([
        # Declare the use_sim_time parameter
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Robot State Publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'publish_frequency': 50.0,
                'frame_prefix': '',
            }],
            arguments=[urdf_file]),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate': 50,
            }]
        ),

        # Optional: TF2 Static Transform Publisher for map->odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])