#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TwistStampedPublisher(Node):
    def __init__(self):
        super().__init__('twist_stamped_pub')
        self.publisher_ = self.create_publisher(TwistStamped, '/nhatbot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)  # 10Hz
        self.get_logger().info("TwistStamped publisher started!")

    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Ví dụ: tiến thẳng 0.2 m/s, quay 0.5 rad/s
        msg.twist.linear.x = 0.2
        msg.twist.angular.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: v={msg.twist.linear.x}, ω={msg.twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
