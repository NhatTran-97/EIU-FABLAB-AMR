import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    use_sim_time = LaunchConfiguration("use_sim_time")
 
    lifecycle_nodes = ["waypoint_follower"]   # 

    

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false",)

   

    waypoint_follower_node =Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("nhatbot_planner"), "config", "waypoint_follower.yaml"), {"use_sim_time": use_sim_time}]
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
        waypoint_follower_node,
        nav2_lifecycle_manager,
        


    ])
