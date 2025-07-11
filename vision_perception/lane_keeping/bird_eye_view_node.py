#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class BirdEyeViewNode(Node):
    def __init__(self):
        super().__init__('bird_eye_view_node')
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.clicked_points = []
        self.matrix_ready = False
        self.matrix = None
        self.output_size = None
        self.window_name = "Original"

        # Load điểm đã lưu nếu có
        if os.path.exists("birdview_points.npy"):
            self.clicked_points = list(np.load("birdview_points.npy"))
            self.get_logger().info("✅ Loaded birdview_points.npy")

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.clicked_points) < 4:
            self.clicked_points.append((x, y))
            self.get_logger().info(f"🖱️ Clicked: {x}, {y}")
            if len(self.clicked_points) == 4:
                np.save("birdview_points.npy", np.array(self.clicked_points))
                self.get_logger().info("💾 Saved to birdview_points.npy")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_copy = frame.copy()

        # Khởi tạo kích thước ảnh đầu ra
        if self.output_size is None:
            self.output_size = (frame.shape[1], frame.shape[0])  # (width, height)

        # Vẽ điểm đã chọn
        for pt in self.clicked_points:
            cv2.circle(frame_copy, pt, 5, (0, 0, 255), -1)

        # Nếu đủ 4 điểm và chưa có ma trận biến đổi → tạo matrix
        if len(self.clicked_points) == 4 and not self.matrix_ready:
            pts1 = np.float32(self.clicked_points)
            pts2 = np.float32([
                [0, 0],
                [self.output_size[0], 0],
                [0, self.output_size[1]],
                [self.output_size[0], self.output_size[1]]
            ])
            self.matrix = cv2.getPerspectiveTransform(pts1, pts2)
            self.matrix_ready = True
            self.get_logger().info("✅ Perspective matrix created.")

        # Nếu đã có matrix, biến đổi ảnh
        if self.matrix_ready:
            warped = cv2.warpPerspective(frame, self.matrix, self.output_size, flags=cv2.INTER_NEAREST)
            cv2.imshow("Bird's Eye View", warped)

        # Hiển thị ảnh gốc với các điểm
        cv2.imshow(self.window_name, frame_copy)

        # Xử lý phím nhấn
        key = cv2.waitKey(1)
        if key == ord('r'):
            self.clicked_points.clear()
            self.matrix_ready = False
            self.get_logger().info("🔄 Reset clicked points")
        elif key == 27 or key == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BirdEyeViewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
