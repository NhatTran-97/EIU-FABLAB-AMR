import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    nhatbot_planner_pkg = get_package_share_directory("nhatbot_planner")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false")

    
    a_star_node = Node(
        package="nhatbot_planner",
        executable="a_star_planner",
        name="a_star_planner",
        output="screen",
    )



    return LaunchDescription([
        a_star_node

    ])