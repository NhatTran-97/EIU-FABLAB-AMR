#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistRelay(Node):
    def __init__(self):
        super().__init__("twist_relay")
        self.controller_sub = self.create_subscription(Twist, "/nhatbot_controller/cmd_vel_unstamped", self.controller_twist_callback, 10)
        self.controoler_pub = self.create_publisher(TwistStamped, "/nhatbot_controller/cmd", 10)
        
        self.joy_sub = self.create_subscription(TwistStamped, "/teleop_stamped", self.joy_twist_callback, 10)
        self.joy_pub = self.create_publisher(Twist, "teleop", 10)

    def controller_twist_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg 
        self.controoler_pub.publish(twist_stamped)
    
    def joy_twist_callback(self, msg):
        twist = Twist()
        twist = msg.twist
        self.joy_pub.publish(twist)

def main():
    rclpy.init()
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

