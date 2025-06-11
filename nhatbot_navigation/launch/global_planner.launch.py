import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Default path to planner_server.yaml inside nav2 package
    default_params_path = os.path.join(
        get_package_share_directory('nav2'),
        'config',
        'planner_server.yaml'
    )

    # Declare a launch argument for external override
    planner_params_file_arg = DeclareLaunchArgument(
        'planner_params_file',
        default_value=default_params_path,
        description='Path to the planner_server and global_costmap parameters YAML file'
    )

    # Substitution object to use the declared argument
    planner_params = LaunchConfiguration('planner_params_file')

    # --- Global Costmap Node (required for planner_server to work) ---
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[planner_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', '/scan')  # Adjust if your LiDAR topic is different
        ]
    )

    # --- Planner Server Node ---
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_params]
    )

    # --- Lifecycle Manager (for both planner and global costmap) ---
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['planner_server']   # global_costmap
         }]
    )

    return LaunchDescription([
        planner_params_file_arg,
        planner_server,
        lifecycle_manager
    ])
