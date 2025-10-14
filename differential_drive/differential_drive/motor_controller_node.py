#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import sys
import signal


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller_node")

      
        self.declare_parameter("wheel_radius", 0.0535)
        self.declare_parameter("wheel_separation", 0.45)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel_radius: {self.wheel_radius}, wheel_separation: {self.wheel_separation}")


        self.vel_sub = self.create_subscription(
            Twist, "nhatbot_controller/cmd_vel", self.vel_callback, 10
        )


        self.wheel_cmd_pub = self.create_publisher(
            Float32MultiArray, "nhatbot_controller/wheel_rotational_vel", 10
        )

    def vel_callback(self, msg):
        """Chuyển đổi vận tốc từ cmd_vel thành tốc độ bánh xe"""
        if any(np.isnan([msg.linear.x, msg.angular.z])) or any(np.isinf([msg.linear.x, msg.angular.z])):
            self.get_logger().warn("Received invalid cmd_vel data! Ignoring this update.")
            return

        # left_wheel_speed = (msg.linear.x + (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius
        # right_wheel_speed = (msg.linear.x - (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius
        left_wheel_speed = (msg.linear.x - (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius
        right_wheel_speed = (msg.linear.x + (msg.angular.z * self.wheel_separation / 2)) / self.wheel_radius


        wheel_speed_msg = Float32MultiArray()
        wheel_speed_msg.data = [left_wheel_speed, right_wheel_speed]

        self.wheel_cmd_pub.publish(wheel_speed_msg)

def main():
    rclpy.init()
    node = MotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok(): 
            node.get_logger().info("🛑 Ctrl+C detected! Đang dừng MotorController Node...")

    finally:
        if rclpy.ok(): 
            node.get_logger().info("✅ MotorController Node đã dừng hoàn toàn!")
        
        node.destroy_node()

        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()





