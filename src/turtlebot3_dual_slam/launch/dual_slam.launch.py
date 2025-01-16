#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = get_package_share_directory('turtlebot3_dual_slam')
    urdf_file = os.path.join(pkg_share, 'urdf', 'turtlebot3_dual_lidar_standalone.urdf.xacro')

    return LaunchDescription([
        # Simulation Time Argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': use_sim_time
            }]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate': 50
            }]
        ),

        # Static Transforms for Sensors and Links
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0.010', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar_2d',
            arguments=['-0.064', '0', '0.122', '0', '0', '0', 'base_link', 'base_scan']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar_3d',
            arguments=['-0.064', '0', '0.222', '0', '0', '0', 'base_link', 'lidar_3d']
        ),

        # SLAM Toolbox for 2D SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'publish_period_sec': 0.02,
                # Ensure correct topic for 2D LiDAR
                'subscribe_scan_topic': '/scan'
            }]
        ),

        # Add a 3D SLAM Node (e.g., hdl_graph_slam)
        Node(
            package='hdl_graph_slam',
            executable='hdl_graph_slam_node',
            name='hdl_graph_slam_node',
            output='screen',
            parameters=[{
                # Use appropriate configuration file for hdl_graph_slam
                "use_sim_time": use_sim_time,
                "lidar_topic": "/lidar_3d"
            }]
        ),

        # Teleop Keyboard Control
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen'
        ),

        # RViz Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'dual_slam.rviz')],
        ),

        # Gazebo Simulation Environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_gazebo'),
                '/launch/turtlebot3_world.launch.py'
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Navigation Stack Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # Ensure the params file exists and is configured correctly
                'params_file': os.path.join(get_package_share_directory('nav2_bringup'), 
                                            'params/nav2_params.yaml')
             }.items()
         ),
    ])
