import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    

    safety_stop_node = Node(
        package='nhatbot_safety',
        executable='safety_stop',
        name='safety_stop_node',
        output='screen',
        parameters=[{
            'danger_distance': 0.25,
           'warning_distance': 0.6,}])
    
    audio_server_node = Node(
        package='peripheral_interfaces',
        executable='audio_server.py',
        name='voice_player_server',
        output='screen',)

    return LaunchDescription([
            safety_stop_node,
            audio_server_node
    ])





