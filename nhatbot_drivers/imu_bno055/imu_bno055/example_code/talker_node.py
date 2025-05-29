#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import signal

class TalkerNode(Node):
    def __init__(self):
        super().__init__("talker_node")
        self.publisher_ = self.create_publisher(String, 'topic',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f"Hello everyone {self.count}"
        self.publisher_.publish(msg)
        self.count += 1
        self.get_logger().info(f"Publishing {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    talkerNode = TalkerNode()

    def shutdown_handler(signum, frame):
        talkerNode.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)
    rclpy.spin(talkerNode)

       


if __name__ == '__main__':
    main()