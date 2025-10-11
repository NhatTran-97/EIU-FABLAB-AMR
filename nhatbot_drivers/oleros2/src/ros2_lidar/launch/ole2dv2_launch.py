#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name

from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
from launch_ros.actions import Node
import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ros2_lidar')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'lidar_driver'

    nhatbot_stack_pkg = get_package_share_directory("ros2_lidar")
    filter_config_path = os.path.join(nhatbot_stack_pkg, "config", "angular_filter.yaml")

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join( share_dir, 'params', 'ole2dv2.yaml'), description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='ros2_lidar',
                                executable='lidar_driver',
                                name=node_name,
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',)


    lidar_tf_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.195', '--y', '0.0', '--z', '0.0557', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser'],
            name='lidar_tf_pub') 
    
    lidar_filter_node = Node(package='laser_filters',
            executable='scan_to_scan_filter_chain',
            parameters=[filter_config_path],
            # remappings=[('/scan', '/scan')]  
        )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] lidar driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

   # TODO make lifecycle transition to shutdown before SIGINT
    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name=node_name),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] lidar driver node is exiting."),
            ],
        )
    )
    # shutdown_event = RegisterEventHandler(
    #     OnShutdown(
    #         on_shutdown=[
    #             EmitEvent(event=ChangeState(
    #             lifecycle_node_matcher=matches_node_name(node_name),  # chỉ truyền tên node
    #             transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
    #             )),
    #             LogInfo(
    #                 msg="[LifecycleLaunch] lidar driver node is exiting."),
    #         ],
    #     )
    # )

    return LaunchDescription([
        params_declare,
        driver_node,
        lidar_tf_node,
        lidar_filter_node,
        activate_event,
        configure_event,
        shutdown_event,
    ])
