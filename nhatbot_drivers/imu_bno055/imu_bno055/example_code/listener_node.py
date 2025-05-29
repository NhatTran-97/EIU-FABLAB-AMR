#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import signal

class ListennerNode(Node):
    def __init__(self):
        super().__init__("talker_node")
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
    def listener_callback(self, msg):
        self.get_logger().info(f"Received {msg.data}")

def main(args = None):
    rclpy.init(args=args)
    listenerNode = ListennerNode()

    def shutdown_handler(signum, frame):
        listenerNode.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)
    rclpy.spin(listenerNode)

    
if __name__=="__main__":
    main()