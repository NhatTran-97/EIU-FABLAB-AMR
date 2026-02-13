import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    use_sim_time = LaunchConfiguration("use_sim_time")
 
    # lifecycle_nodes = ["planner_server","smoother_server", "controller_server", "behavior_server", "bt_navigator"] 
    lifecycle_nodes = ["planner_server","smoother_server", "controller_server", "bt_navigator"]   #     
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false",)

    compute_path_node =Node(
        package='nhatbot_navigation',
        executable='compute_path_client',
        name='compute_path_client',
        output='screen',)
    
    follow_path_node =Node(
        package='nhatbot_navigation',
        executable='follow_path_client',
        name='follow_path_client',
        output='screen',)

    waypoint_follower_node =Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("nhatbot_planner"), "config", "waypoint_follower.yaml"), {"use_sim_time": use_sim_time}]
    )

    global_costmap =Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("nhatbot_planner"), "config", "costmap.yaml"), {"use_sim_time": use_sim_time}])



    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("nhatbot_planner"), "config", "planner_server.yaml"), {"use_sim_time": use_sim_time}],)

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("nhatbot_planner"),"config", "smoother_server.yaml"), {"use_sim_time": use_sim_time}],)
    
    controller_server =Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("nhatbot_controller"), "config", "controller_server.yaml"), {"use_sim_time": use_sim_time}])

    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("nhatbot_behavior"),"config", "bt_navigator.yaml"), {"use_sim_time": use_sim_time}],)
  
    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("nhatbot_behavior"),"config", "behavior_server.yaml"), {"use_sim_time": use_sim_time}],)

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
        # compute_path_node,follow_path_node, 
        nav2_planner_server,
        nav2_smoother_server,
        controller_server,
        # nav2_behaviors,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
        


    ])
