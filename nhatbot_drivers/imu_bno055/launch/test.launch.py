from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    talker = Node(
        package='imu_bno055',
        executable='talkerNode_exe',
        name='talker_node'
    )
    listener = Node(
        package='imu_bno055',
        executable='listenerNode_exe',
        name='listener_node'
    )

    return LaunchDescription([
        talker,
        listener
    ])