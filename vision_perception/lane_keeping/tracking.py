#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import cv2
import tf_transformations
from tf_transformations import euler_from_quaternion


class PurePursuitVisualizer(Node):
    def __init__(self):
        super().__init__('pure_pursuit_visualizer')
        self.sub_path = self.create_subscription(Path, "/lane_center_path", self.path_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/ackermann_drive", 10)

        self.path_points = []
        self.lookahead_distance = 0.6  # meters
        self.odom_position = None
        self.odom_yaw = 0.0

        self.timer = self.create_timer(0.05, self.run_pure_pursuit)

    def path_callback(self, msg):
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.odom_position = (pos.x, pos.y)

        quat = msg.pose.pose.orientation
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.odom_yaw = yaw

    def run_pure_pursuit(self):
        if self.odom_position is None or len(self.path_points) < 2:
            return

        car_x, car_y = self.odom_position

        # Tìm điểm nhìn xa (Lookahead point)
        target_point = None
        for pt_x, pt_y in self.path_points:
            dist = np.hypot(pt_x - car_x, pt_y - car_y)
            if dist >= self.lookahead_distance:
                target_point = (pt_x, pt_y)
                break

        if target_point is None:
            self.get_logger().warn("🚫 No valid lookahead point found!")
            return

        # Tính góc điều khiển theo Pure Pursuit
        dx = target_point[0] - car_x
        dy = target_point[1] - car_y

        # Chuyển về tọa độ xe
        local_x = np.cos(self.odom_yaw) * dx + np.sin(self.odom_yaw) * dy
        local_y = -np.sin(self.odom_yaw) * dx + np.cos(self.odom_yaw) * dy

        # Tránh chia 0
        if local_x == 0:
            return

        curvature = (2 * local_y) / (self.lookahead_distance ** 2)
        steering_angle = np.arctan(curvature)

        # Gửi điều khiển
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.5  # m/s
        msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(msg)

        # Log
        self.get_logger().info(f"🚗 Steering: {np.degrees(steering_angle):.2f}° | Speed: {msg.drive.speed:.2f} m/s")

        # Optional: vẽ bằng OpenCV nếu bạn muốn debug
        self.visualize_debug(car_x, car_y, target_point)

    def visualize_debug(self, car_x, car_y, target_point):
        img = np.zeros((500, 500, 3), dtype=np.uint8)
        scale = 50  # 1m = 50px

        def to_pixel(x, y):
            px = int(250 + x * scale)
            py = int(500 - y * scale)
            return (px, py)

        for pt in self.path_points:
            cv2.circle(img, to_pixel(*pt), 2, (0, 255, 255), -1)

        cv2.circle(img, to_pixel(car_x, car_y), 5, (255, 255, 255), -1)
        cv2.circle(img, to_pixel(*target_point), 6, (255, 0, 0), -1)

        cv2.line(img, to_pixel(car_x, car_y), to_pixel(*target_point), (0, 255, 0), 2)
        cv2.imshow("Pure Pursuit Debug", img)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
