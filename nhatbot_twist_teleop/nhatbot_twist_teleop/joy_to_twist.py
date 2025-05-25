#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('joy_to_twist')

        self.declare_parameter("linear_scale", 0.5)          # Hệ số tốc độ tiến/lùi
        self.declare_parameter("angular_scale", 0.5)         # Hệ số tốc độ quay
        self.declare_parameter("deadman_button", 9)          # Nút giữ để di chuyển
        self.declare_parameter("deadzone_threshold", 0.1)    # Ngưỡng tối thiểu tránh trôi
        self.declare_parameter("axis_linear_x", 1)          # Trục điều khiển tiến/lùi
        self.declare_parameter("axis_angular_z", 2)         # Trục điều khiển quay trái/phải

        # Lấy giá trị từ tham số ROS 2
        self.linear_scale = self.get_parameter("linear_scale").get_parameter_value().double_value
        self.angular_scale = self.get_parameter("angular_scale").get_parameter_value().double_value
        self.deadman_button = self.get_parameter("deadman_button").get_parameter_value().integer_value
        self.deadzone_threshold = self.get_parameter("deadzone_threshold").get_parameter_value().double_value
        self.axis_linear_x = self.get_parameter("axis_linear_x").get_parameter_value().integer_value
        self.axis_angular_z = self.get_parameter("axis_angular_z").get_parameter_value().integer_value

        # Subscribe vào topic /joy
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publisher gửi dữ liệu lên /teleop
        self.twist_publisher = self.create_publisher(Twist, '/nhatbot_controller/cmd_vel', 10)  # "nhatbot_controller/cmd_vel   teleop

        self.get_logger().info("✅ joy_to_twist đã khởi động với tham số từ ROS 2!")

    def joy_callback(self, joy_msg):
        """ Xử lý dữ liệu từ joystick và xuất ra Twist """
        twist = Twist()

        # Kiểm tra nếu không nhấn nút deadman -> dừng robot
        if joy_msg.buttons[self.deadman_button] == 0:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # Đọc giá trị từ joystick
            linear_value = joy_msg.axes[self.axis_linear_x]
            angular_value = joy_msg.axes[self.axis_angular_z]

            # Nếu giá trị trục nhỏ hơn ngưỡng, set về 0 để tránh trôi nhẹ
            if abs(linear_value) < self.deadzone_threshold:
                linear_value = 0.0
            if abs(angular_value) < self.deadzone_threshold:
                angular_value = 0.0

            # Tính toán vận tốc
            twist.linear.x = linear_value * self.linear_scale
            twist.angular.z = angular_value * self.angular_scale

        # Xuất dữ liệu lên topic /teleop
        self.twist_publisher.publish(twist)

def main():
    rclpy.init()
    node = JoyToTwistNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("🛑 Ctrl+C detected! Đang dừng joy_twist_node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()



