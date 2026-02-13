import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    use_sim_time = LaunchConfiguration("use_sim_time")
 
    lifecycle_nodes = ["controller_server"]   
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false",)


    controller_server =Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("nhatbot_controller"), "config", "controller_server.yaml"), {"use_sim_time": use_sim_time}]
    )
    follow_path =Node(
        package='nhatbot_navigation',
        executable='follow_path_client',
        name='follow_path_client',
        output='screen',
    )


    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )




    return LaunchDescription([
        use_sim_time_arg,
        controller_server,
        follow_path,
        nav2_lifecycle_manager,
        


    ])
