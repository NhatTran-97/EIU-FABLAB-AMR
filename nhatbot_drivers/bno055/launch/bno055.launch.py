import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('bno055'),
        'config',
        'bno055_params_i2c.yaml'
    )
        
    imu_node = Node(
        package='bno055',
        executable='bno055',
        parameters=[config],
        output='screen'
    )

    imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.175', '-0.048', '0.041', '0', '0', '0', 'base_link', 'imu_link'],
        name='imu_tf_pub'
    )

    ld.add_action(imu_node)
    ld.add_action(imu_tf_node)
    return ld
