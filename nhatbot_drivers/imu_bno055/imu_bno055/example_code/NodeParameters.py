#!/usr/bin/env python3

from rclpy.node import Node
#from connectors.uart import UART

class Parameters:
    """
    ROS2 Node Parameter Handling
    """
    def __init__(self,node:Node):
        node.get_logger().info('Initializing parameters')

        node.declare_parameter(name='ros_topic_prefix', value='bno055/')
        node.declare_parameter(name='connection_type', value='uart')
        node.declare_parameter('uart_port', value='/dev/ttyUSB0')
        node.declare_parameter('uart_baudrate', value=115200)
        node.declare_parameter('uart_timeout', value=0.1)