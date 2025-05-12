#!/usr/bin/env python3

import rclpy  
from rclpy.node import Node 
import numpy as np 

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):
    """The class that handles emergency braking"""

    def __init__(self):
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message

        You should also subscribe to the scan topic to get the LaserScan message and the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_calback and scan_calback as callback methods  

        NOTE that the x component of the linear velocity odom is the speed

        """
        self.speed = 0.0

        # Create ROS subscribers and publishers.
        self.scan_subscription = self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 1000)
        self.teleop_publisher_ = self.create_publisher(AckermannDriveStamped, 'teleop', 1000)
        self.bool_publisher_ = self.create_publisher(Bool, 'emergency_breaking', 1000)


    def odom_callback(self, odom_msg):
        # Update current speed
        self.speed = odom_msg.twist.twist.linear.x
    
    def scan_callback(self, scan_msg):
        # Calculate TTC
        emergency_breaking = False 

        for idx, r in enumerate(scan_msg.ranges):

            if(np.isnan(r) or r > scan_msg.range_max or r < scan_msg.range_min): 
                continue
            threshold = 0.4

            if r / max(self.speed * np.cos(scan_msg.angle_min + (idx * scan_msg.angle_increment)), 0.001) < threshold:
                emergency_breaking = True
                break
        
        emergency_msg = Bool()
        emergency_msg.data = emergency_breaking

        # Publish command to brake 

        if emergency_breaking:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            self.get_logger().info("emergency brake engaged at speed {}".format(self.speed))
            
            self.publisher_.publish(drive_msg) # for autonomous control 
            self.teleop_publisher_.publish(drive_msg) # for manual control
        
        self.bool_publisher_.publish(emergency_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
