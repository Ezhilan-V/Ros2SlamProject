from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        Node(
            package='robot_control',
            executable='keyboard_control.py',
            name='keyboard_control',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])