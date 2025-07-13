from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from math import atan2

class CenterlineFollower(Node):
    def __init__(self):
        super().__init__("centerline_follower")
        self.path_sub = self.create_subscription(Path, "/lane_center_path", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        
        target_pose = msg.poses[min(10, len(msg.poses)-1)]  # chọn điểm lookahead
        dx = target_pose.pose.position.x
        dy = target_pose.pose.position.y

        angle = atan2(dy, dx)
        distance = (dx**2 + dy**2)**0.5

        cmd = Twist()
        cmd.linear.x = 1.0  # tốc độ cố định
        cmd.angular.z = angle  # hoặc Kp * angle nếu cần
        self.cmd_pub.publish(cmd)
