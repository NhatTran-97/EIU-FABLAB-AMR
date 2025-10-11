import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="False",
                                      description="Use simulated time")

    # Declare launch arguments
    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.0535")
    wheel_separation_arg = DeclareLaunchArgument("wheel_separation", default_value="0.45")
    use_simple_controller_arg = DeclareLaunchArgument("use_simple_controller", default_value="True")
    use_bno055_arg = DeclareLaunchArgument("use_bno055", default_value="True")

    use_python_arg = DeclareLaunchArgument("use_python",default_value="False")
    use_rviz_arg = DeclareLaunchArgument("use_rviz",default_value="True")
    use_localize_arg = DeclareLaunchArgument("use_localization",default_value="True")

    # Launch configurations
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    use_python = LaunchConfiguration("use_python")
    use_rviz = LaunchConfiguration("use_rviz")
    use_localization = LaunchConfiguration("use_localization")
    use_bno055 = LaunchConfiguration("use_bno055")

    # Package directories
    nhatbot_stack_pkg = get_package_share_directory("nhatbot_stack")
    twist_mux_pkg = get_package_share_directory("twist_mux")
    rosbridge_pkg = get_package_share_directory("rosbridge_server")
    bno055_pkg = get_package_share_directory("bno055")
    firmware_pkg = get_package_share_directory("nhatbot_firmware")
    lidar_pkg = get_package_share_directory("ros2_lidar")

    # Joy node (joystick driver)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(nhatbot_stack_pkg, "config", "joy_config.yaml")])


    twist_relay_node = Node(
        package="nhatbot_twist_teleop",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}])
    
    # Twist Teleop node
    joy_to_twist = Node(
        package="nhatbot_twist_teleop",
        executable="joy_to_twist",
        name="joy_to_twist",
        output="screen",
        parameters=[
            {"linear_scale": 0.3},        
            {"angular_scale": 0.3},        
            {"deadman_button": 9},          
            {"deadzone_threshold": 0.01}  ])

    # Include twist_mux launch
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(twist_mux_pkg, "launch", "twist_mux_launch.py")),
        launch_arguments={"cmd_vel_out": "/nhatbot/cmd_vel_unstamped",
            "config_topics": os.path.join(nhatbot_stack_pkg, "config", "twist_mux_topics.yaml"),
            "config_locks": os.path.join(nhatbot_stack_pkg, "config", "twist_mux_locks.yaml"),
            "config_joy":  os.path.join(nhatbot_stack_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time":  LaunchConfiguration("use_sim_time")}.items())
    
    bno055_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bno055_pkg, "launch", "bno055.launch.py")),
        condition=IfCondition(use_bno055))

    # ZLAC8015D driver
    zlac_driver = Node(
        package="zlac8015d_driver",
        executable="zlac_interface_node.py",
        name="zlac_driver_node",
        output="screen")
    

    velocity_controller_node_py = Node(
        package="differential_drive",
        executable="differential_drive_controller.py",
        arguments=[{"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,}],
        condition=IfCondition(use_python))


    velocity_controller_node_cpp =  Node(
            package="differential_drive",
            executable="motor_controller_node",
            name="motor_controller",
            output="screen",
            parameters=[{
                "wheel_radius": wheel_radius,
                "wheel_separation": wheel_separation,
                "linear_velocity_max_": 0.5,
                "angular_velocity_max_": 0.5}],
                condition=UnlessCondition(use_python))
    
    odom_estimator_node = Node(
            package="differential_drive",
            executable="odom_estimator_node",
            name="odom_estimator",
            output="screen",
            emulate_tty=True,
            parameters=[{"wheel_radius":wheel_radius, "wheel_separation": wheel_separation, "enable_tf_broadcast:": True}])



    # Include rosbridge server launch
    rosbridge_launch = IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(rosbridge_pkg, "launch", "rosbridge_websocket_launch.xml")))

    # Call service reset_feedback_position
    reset_feedback_service = ExecuteProcess(cmd=["ros2", "service", "call", "/SensorBroadcaster/reset_encoder", "std_srvs/srv/Trigger"],output="screen")

    # Call service reset_odom
    reset_odom_service = ExecuteProcess(cmd=["ros2", "service", "call", "/reset_odom", "std_srvs/srv/Trigger"], output="screen")

    # lidar_node = IncludeLaunchDescription(os.path.join(get_package_share_directory("nhatbot_stack"),"launch", "lidar_a1_filter.launch.py"),)
    a_star_node = IncludeLaunchDescription(os.path.join(get_package_share_directory("nhatbot_planner"),"launch", "nhatbot_planner.launch.py"),)
    localization_node = IncludeLaunchDescription(os.path.join(get_package_share_directory("nhatbot_stack"),"launch", "robot_localization.launch.py"),condition=IfCondition(use_localization))
    utils_nodes = IncludeLaunchDescription(os.path.join(nhatbot_stack_pkg,"launch", "utils.launch.py"),)

    hw_interface_node = IncludeLaunchDescription(os.path.join(firmware_pkg,"launch", "bringup_hardware_interface.launch.py"),)

    lidar_node = IncludeLaunchDescription(os.path.join(lidar_pkg,"launch","ole2dv2_launch.py"),)

    
    usb_cam = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[os.path.join(nhatbot_stack_pkg, "config", "usb_params.yaml")])


    micro_ros_node =   Node(
                    package='micro_ros_agent',
                    executable='micro_ros_agent',
                    name='micro_ros_serial_agent',
                    output='screen',
                    arguments=['serial', '--dev', '/dev/esp_device',  '-b', '115200']) 
                                            

    static_pub =  Node(
                package='nhatbot_stack',
                executable='static_tf_pub_node',
                output='screen')
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(nhatbot_stack_pkg, "rviz", "nav2_default_view.rviz")],
        condition=IfCondition(use_rviz))



    # Return the LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        use_bno055_arg, 
        wheel_radius_arg,
        use_rviz_arg,
        use_localize_arg, 
        wheel_separation_arg,
        use_simple_controller_arg,
        use_python_arg,
        joy_node,
        twist_relay_node,
        joy_to_twist,
        twist_mux_launch,
        hw_interface_node, 
        reset_feedback_service,
        bno055_launch,
        static_pub,
        lidar_node,




        #velocity_controller_node_py,
        #velocity_controller_node_cpp,
        #odom_estimator_node,
        # rosbridge_launch,
        
      
       # localization_node,
        # a_star_node,
        # usb_cam,
    #    utils_nodes,
    
        # micro_ros_node, 

     #   rviz_node,
      
    ])
