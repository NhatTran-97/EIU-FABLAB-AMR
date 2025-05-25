import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="False",
                                      description="Use simulated time"
    )

    # Declare launch arguments
    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.0535")
    wheel_separation_arg = DeclareLaunchArgument("wheel_separation", default_value="0.45")
    use_simple_controller_arg = DeclareLaunchArgument("use_simple_controller", default_value="True")

    # Launch configurations
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    # Package directories
    nhatbot_stack_pkg = get_package_share_directory("nhatbot_stack")
    twist_mux_pkg = get_package_share_directory("twist_mux")
    rosbridge_pkg = get_package_share_directory("rosbridge_server")

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
            {"linear_scale": 0.5},        
            {"angular_scale": 0.5},        
            {"deadman_button": 9},          
            {"deadzone_threshold": 0.01}  
        ]

    )

    # Include twist_mux launch
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(twist_mux_pkg, "launch", "twist_mux_launch.py")
        ),
        launch_arguments={
            "cmd_vel_out": "/nhatbot/cmd_vel_unstamped",
            "config_topics": os.path.join(nhatbot_stack_pkg, "config", "twist_mux_topics.yaml"),
            "config_locks": os.path.join(nhatbot_stack_pkg, "config", "twist_mux_locks.yaml"),
            "config_joy":  os.path.join(nhatbot_stack_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time":  LaunchConfiguration("use_sim_time")
        }.items()
    )

    # ZLAC8015D driver
    zlac_driver = Node(
        package="zlac8015d_driver",
        executable="zlac_interfacer.py",
        name="zlac_driver_node",
        output="screen"
    )
    

    differential_drive_controller = Node(
        package="differential_drive",
        executable="differential_drive_controller.py",
        arguments=[{"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,}],
        condition=IfCondition(use_simple_controller)
    )


    motor_controller_node =  Node(
            package="differential_drive",
            executable="motor_controller_node",
            name="motor_controller",
            output="screen",
            parameters=[{
                "wheel_radius": wheel_radius,
                "wheel_separation": wheel_separation,
                "linear_velocity_max_": 0.5,
                "angular_velocity_max_": 0.5
            }]
        )
    odom_estimator_node = Node(
            package="differential_drive",
            executable="odom_estimator_node",
            name="odom_estimator",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "wheel_radius":wheel_radius, "wheel_separation": wheel_separation, "enable_tf_broadcast:": True}])

    # Include rosbridge server launch
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_pkg, "launch", "rosbridge_websocket_launch.xml")
        )
    )

    # Call service reset_feedback_position
    reset_feedback_service = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/nhatbot/reset_feedback_position",
            "std_srvs/srv/Trigger"
        ],
        output="screen"
    )

    # Call service reset_odom
    reset_odom_service = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/reset_odom",
            "std_srvs/srv/Trigger"
        ],
        output="screen"
    )


    lidar_node = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("nhatbot_stack"),"launch", "lidar_a1_filter.launch.py"),)
    

    static_pub =  Node(
                package='nhatbot_stack',
                executable='static_tf_pub_node',
                output='screen')
            
    # Return the LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        joy_node,
        twist_relay_node,
        joy_to_twist,
        twist_mux_launch,
        # zlac_driver,
        # differential_drive_controller,
        motor_controller_node,
        odom_estimator_node,
        rosbridge_launch,
        reset_feedback_service,
        lidar_node,
        # static_pub
        # reset_odom_service
        # reset_odom_handler
    ])
