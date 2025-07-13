#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nhatbot_msgs.msg import LaneSteering
from ackermann_msgs.msg import AckermannDriveStamped

class LaneSteeringController(Node):
    def __init__(self):
        super().__init__('lane_steering_controller')
        self.sub = self.create_subscription(LaneSteering, '/lane_steering', self.lane_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/cmd_drive', 10)

        self.kp = 1.0  # Hệ số điều chỉnh, bạn có thể chỉnh theo thực tế
        self.speed = 1.0  # m/s

    def lane_callback(self, msg: LaneSteering):
        angle_error = msg.angle_rad  # Giả sử angle_rad = lệch giữa hướng xe và lane center

        steering = self.kp * angle_error

        # Giới hạn góc đánh lái
        max_angle = 0.4  # radians, khoảng ~23 độ
        steering = max(min(steering, max_angle), -max_angle)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = self.speed
        self.pub.publish(drive_msg)

        self.get_logger().info(f"▶️ Steering: {steering:.3f} rad, Speed: {self.speed:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = LaneSteeringController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
