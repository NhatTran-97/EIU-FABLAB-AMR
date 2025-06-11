import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('nhatbot_planner')
    costmap_yaml_path = os.path.join(package_share_directory, 'config', 'costmap.yaml')

    costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        namespace='',
        output='screen',
        parameters=[costmap_yaml_path]
    )

    # Gửi configure (sau 2 giây)
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: True,  # Khớp tất cả để đảm bảo hoạt động
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # Gửi activate (sau 4 giây)
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: True,
            transition_id=Transition.TRANSITION_ACTIVATE
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=costmap_yaml_path,
            description='Path to costmap.yaml'
        ),
        costmap_node,
        TimerAction(period=2.0, actions=[configure_event]),
        TimerAction(period=4.0, actions=[activate_event])
    ])
