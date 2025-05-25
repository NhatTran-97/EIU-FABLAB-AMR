#!/usr/bin/env python3

import math
import numpy as np
import signal
import sys
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration

class OdomEstimator(Node):
    """
    Node 2: odom_estimator_node ➝ Tính toán Odom
    * Nhận encoder từ /nhatbot_firmware/JointState
    * Tính toán Odom từ encoder
    * Xuất Odom ra /nhatbot_controller/odom
    * Broadcast TF (odom → base_link)
    * Cung cấp service /reset_odom để reset vị trí robot
    """
    def __init__(self):
        super().__init__("odom_estimator_node")

        # Load tham số từ ROS2 parameter
        self.declare_parameter("wheel_radius", 0.0535)
        self.declare_parameter("wheel_separation", 0.45)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        # Biến lưu trạng thái Odom
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.prev_time_ = self.get_clock().now()

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0

        self.joint_sub_ = self.create_subscription(JointState, "nhatbot_firmware/JointState", self.joint_callback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "nhatbot_controller/odom", 10)


        # TF broadcaster
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()

    def joint_callback(self, msg):
        """Nhận dữ liệu encoder và cập nhật Odom"""
        if hasattr(self, "block_update_until") and self.get_clock().now() < self.block_update_until:
            return

        if not msg.position or len(msg.position) < 2:
            return

        if any(math.isnan(pos) or math.isinf(pos) for pos in msg.position):
            return

        # Tính dt (delta thời gian giữa hai lần cập nhật)
        current_time = Time.from_msg(msg.header.stamp)
        dt = max((current_time - self.prev_time_).nanoseconds / 1e9, 1e-6)  # Đảm bảo dt >= 1µs
        self.prev_time_ = current_time

        if msg.position[0] == 0.0 and msg.position[1] == 0.0:
            if self.left_wheel_prev_pos_ != 0.0 or self.right_wheel_prev_pos_ != 0.0:
                self.reset_odom()
            return 

        dp_left = msg.position[0] - self.left_wheel_prev_pos_
        dp_right = msg.position[1] - self.right_wheel_prev_pos_

        # Ngưỡng bỏ qua nhiễu nhỏ
        if abs(dp_left) < 0.0005 and abs(dp_right) < 0.0005:
            return 

        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]

        phi_left = dp_left / dt
        phi_right = dp_right / dt

        self.update_odometry(phi_left, phi_right, dt)

    def update_odometry(self, phi_left, phi_right, dt):
        """ Cập nhật Odom từ tốc độ góc của bánh xe """
        linear_velocity = self.wheel_radius * (phi_right + phi_left) / 2.0
        angular_velocity = self.wheel_radius * (phi_right - phi_left) / self.wheel_separation

        alpha = 0.2
        self.linear_filtered = alpha * linear_velocity + (1 - alpha) * getattr(self, "linear_filtered", linear_velocity)
        self.angular_filtered = alpha * angular_velocity + (1 - alpha) * getattr(self, "angular_filtered", angular_velocity)

        if abs(self.linear_filtered) < 0.001:
            self.linear_filtered = 0.0
        if abs(self.angular_filtered) < 0.001:
            self.angular_filtered = 0.0

        self.x_ += self.linear_filtered * dt * math.cos(self.theta_)
        self.y_ += self.linear_filtered * dt * math.sin(self.theta_)
        self.theta_ += self.angular_filtered * dt
        self.theta_ = math.atan2(math.sin(self.theta_), math.cos(self.theta_))

        self.publish_odom()
        self.publish_transform()

    def publish_transform(self):
        """Broadcast transform from 'odom' to 'base_link'."""
        q = quaternion_from_euler(0, 0, self.theta_)

        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.transform_stamped_.header.frame_id = "odom"  # Thời gian gốc là odom
        self.transform_stamped_.child_frame_id = "base_link"  # Thời gian con là base_link

        # Xử lý vị trí và góc của robot
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0  # Z luôn bằng 0 trong trường hợp 2D

        # Sử dụng quaternions để biểu diễn góc (theta)
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]

        # Broadcast transform
        self.tf_broadcaster_.sendTransform(self.transform_stamped_)



    def reset_odom(self):
        """Reset vị trí Odom và đồng bộ với encoder"""
        self.x_, self.y_, self.theta_ = 0.0, 0.0, 0.0
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()
        self.block_update_until = self.get_clock().now() + Duration(seconds=0.1)


        self.linear_filtered = 0.0
        self.angular_filtered = 0.0

        self.get_logger().info("⚡ Encoder reset detected! Odom reset automatically.")
        self.publish_odom() 

def main():
    rclpy.init()
    node = OdomEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok(): 
            node.get_logger().info("🛑 Ctrl+C detected! Đang dừng odom_estimator_node...")

    finally:
        if rclpy.ok(): 
            node.get_logger().info("✅ Odom estimator node đã dừng hoàn toàn!")
        
        node.destroy_node()

        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()
