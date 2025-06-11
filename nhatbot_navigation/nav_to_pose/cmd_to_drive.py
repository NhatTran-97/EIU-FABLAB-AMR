#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelToDrive(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_drive')
        # Subscriber to /cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # Publisher to /drive topic
        self.drive_pub = self.create_publisher(
            Twist,
            '/nhatbot_controller/cmd_vel',
            10
        )

    def cmd_vel_callback(self, msg):
        # Republish the received msg to the /drive topic
        self.drive_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToDrive()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
