#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path  # Nếu muốn publish đường đi trung tâm
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float32
from nhatbot_msgs.msg import LaneSteering

def nothing(x):
    pass  # Trackbar callback

class BirdEyeViewNode(Node):
    def __init__(self):
        super().__init__('bird_eye_view_node')
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.lane_marker_pub = self.create_publisher(Marker, "/lane_area_marker", 10)
        self.lane_area_pub = self.create_publisher(Marker, "/lane_area_marker_area", 10)
        self.steering_pub = self.create_publisher(LaneSteering, "/lane_steering_angle", 10)


        self.bridge = CvBridge()
        self.clicked_points = []
        self.matrix_ready = False
        self.matrix = None
        self.output_size = None
        self.window_name = "Original"
        
        # Centerline path publisher (nếu muốn dùng với navigation stack)
        self.centerline_pub = self.create_publisher(Path, "/lane_center_path", 10)

        # Load điểm đã lưu
        if os.path.exists("birdview_points.npy"):
            self.clicked_points = list(np.load("birdview_points.npy"))
            self.get_logger().info("✅ Loaded birdview_points.npy")

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        cv2.namedWindow("Trackbars")

        default_values = {"L-H": 0, "L-S": 0, "L-V": 0, "U-H": 255, "U-S": 31, "U-V": 196}
        if os.path.exists("trackbar_values.npy"):
            try:
                saved_values = np.load("trackbar_values.npy", allow_pickle=True).item()
                self.get_logger().info(f"✅ Loaded HSV values: {saved_values}")
            except Exception as e:
                saved_values = default_values
                self.get_logger().warn(f"⚠️ Error loading HSV file: {e}, using default.")
        else:
            saved_values = default_values
            self.get_logger().info("🟡 No HSV config found, using default.")

        # Tạo các thanh trượt HSV
        for key in default_values:
            cv2.createTrackbar(key.replace("-", " - "), "Trackbars", saved_values[key], 255, nothing)

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

        if self.output_size is None:
            self.output_size = (frame.shape[1], frame.shape[0])

        for pt in self.clicked_points:
            cv2.circle(frame_copy, pt, 5, (0, 0, 255), -1)

        if len(self.clicked_points) == 4 and not self.matrix_ready:
            pts1 = np.float32(self.clicked_points)
            pts2 = np.float32([[0, 0], [self.output_size[0], 0], [0, self.output_size[1]], [self.output_size[0], self.output_size[1]]])
            self.matrix = cv2.getPerspectiveTransform(pts1, pts2)
            self.matrix_ready = True
            self.get_logger().info("✅ Perspective matrix created.")

        if self.matrix_ready:
            warped = cv2.warpPerspective(frame, self.matrix, self.output_size)
            hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

            l_h = cv2.getTrackbarPos("L - H", "Trackbars")
            l_s = cv2.getTrackbarPos("L - S", "Trackbars")
            l_v = cv2.getTrackbarPos("L - V", "Trackbars")
            u_h = cv2.getTrackbarPos("U - H", "Trackbars")
            u_s = cv2.getTrackbarPos("U - S", "Trackbars")
            u_v = cv2.getTrackbarPos("U - V", "Trackbars")

            lower = np.array([l_h, l_s, l_v])
            upper = np.array([u_h, u_s, u_v])
            mask = cv2.inRange(hsv, lower, upper)

            # Histogram base
            histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
            midpoint = histogram.shape[0] // 2
            left_base = np.argmax(histogram[:midpoint])
            right_base = np.argmax(histogram[midpoint:]) + midpoint

            # Sliding window
            window_height = 40
            window_width = 50
            y = mask.shape[0]
            left = left_base
            right = right_base
            lx, ly, rx, ry = [], [], [], []
            msk = mask.copy()

            while y > 0:
                # LEFT
                xl1 = max(left - window_width, 0)
                xl2 = min(left + window_width, mask.shape[1])
                roi_left = mask[y - window_height:y, xl1:xl2]

                contours, _ = cv2.findContours(roi_left, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for c in contours:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        lx.append(xl1 + cx)
                        ly.append(y - window_height // 2)
                        left = xl1 + cx

                # RIGHT
                xr1 = max(right - window_width, 0)
                xr2 = min(right + window_width, mask.shape[1])
                roi_right = mask[y - window_height:y, xr1:xr2]

                contours, _ = cv2.findContours(roi_right, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for c in contours:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        rx.append(xr1 + cx)
                        ry.append(y - window_height // 2)
                        right = xr1 + cx

                cv2.rectangle(msk, (xl1, y), (xl2, y - window_height), (255, 255, 255), 2)
                cv2.rectangle(msk, (xr1, y), (xr2, y - window_height), (255, 255, 255), 2)
                y -= window_height

            # Draw lanes

            left_fit = None
            right_fit = None

            # 2. VẼ LANE TRÁI
            if len(lx) >= 5 and len(ly) >= 5:
                left_fit = np.polyfit(ly, lx, 2)
                for y_fit in range(mask.shape[0]):
                    x_fit = int(left_fit[0] * y_fit ** 2 + left_fit[1] * y_fit + left_fit[2])
                    if 0 <= x_fit < mask.shape[1]:
                        cv2.circle(warped, (x_fit, y_fit), 2, (0, 0, 255), -1)
            else:
                self.get_logger().warn("⚠️ Không tìm thấy đủ điểm để fit lane trái")

            # 3. VẼ LANE PHẢI
            if len(rx) >= 5 and len(ry) >= 5:
                right_fit = np.polyfit(ry, rx, 2)
                for y_fit in range(mask.shape[0]):
                    x_fit = int(right_fit[0] * y_fit ** 2 + right_fit[1] * y_fit + right_fit[2])
                    if 0 <= x_fit < mask.shape[1]:
                        cv2.circle(warped, (x_fit, y_fit), 2, (0, 255, 0), -1)
            else:
                self.get_logger().warn("⚠️ Không tìm thấy đủ điểm để fit lane phải")


                    # --- Vẽ vùng lane dưới dạng TRAPEZOID/TRIANGLE ---
    
            if left_fit is not None and right_fit is not None:
                lane_area_marker = Marker()
                lane_area_marker.header = Header()
                lane_area_marker.header.frame_id = "base_link"
                lane_area_marker.header.stamp = self.get_clock().now().to_msg()
                lane_area_marker.ns = "lane_area"
                lane_area_marker.id = 0
                lane_area_marker.type = Marker.TRIANGLE_LIST
                lane_area_marker.action = Marker.ADD
                lane_area_marker.pose.orientation.w = 1.0
                lane_area_marker.scale.x = 1.0 
                lane_area_marker.scale.y = 1.0 
                lane_area_marker.scale.z = 1.0
   
                color = ColorRGBA()
                color.r = 0.0
                color.g = 1.0
                color.b = 0.0
                color.a = 1.0

                scale = 0.005  # đổi pixel → mét
                step = 10
                for i in range(0, mask.shape[0] - step, step):
                    lx_val = int(np.polyval(left_fit, i))
                    rx_val = int(np.polyval(right_fit, i))
                    lx_next = int(np.polyval(left_fit, i + step))
                    rx_next = int(np.polyval(right_fit, i + step))

                    # Tính tọa độ trong base_link (centered theo width)
                    p1 = Point(x=(lx_val - frame.shape[1] / 2) * scale, y=(frame.shape[0] - i) * scale, z=0.0)
                    p2 = Point(x=(rx_val - frame.shape[1] / 2) * scale, y=(frame.shape[0] - i) * scale, z=0.0)
                    p3 = Point(x=(lx_next - frame.shape[1] / 2) * scale, y=(frame.shape[0] - (i + step)) * scale, z=0.0)
                    p4 = Point(x=(rx_next - frame.shape[1] / 2) * scale, y=(frame.shape[0] - (i + step)) * scale, z=0.0)

                    # 2 tam giác: p1-p2-p3 và p3-p2-p4
                    lane_area_marker.points.extend([p1, p2, p3, p3, p2, p4])
                    lane_area_marker.colors.extend([color] * 6)  # mỗi đỉnh cần 1 màu
           
                self.lane_area_pub.publish(lane_area_marker)
        

            # 1. VẼ VÙNG GIỮA 2 LÀN
            if len(lx) >= 5 and len(rx) >= 5:
                lane_pts = []
                for y_fit in range(0, mask.shape[0], 5):
                    x_left = int(np.polyval(left_fit, y_fit))
                    x_right = int(np.polyval(right_fit, y_fit))
                    if 0 <= x_left < mask.shape[1] and 0 <= x_right < mask.shape[1]:
                        lane_pts.append((x_left, y_fit))
                for y_fit in reversed(range(0, mask.shape[0], 5)):
                    x_right = int(np.polyval(right_fit, y_fit))
                    if 0 <= x_right < mask.shape[1]:
                        lane_pts.append((x_right, y_fit))
                
                lane_pts = np.array(lane_pts, np.int32).reshape((-1, 1, 2))
                cv2.fillPoly(warped, [lane_pts], color=(100, 100, 255))  # màu tím nhạt
            



                # === TỪ LANE_PTS (dạng pixel) → marker hình học ===
                lane_marker = Marker()
                lane_marker.header = Header()
                lane_marker.header.frame_id = "base_link"
                lane_marker.header.stamp = self.get_clock().now().to_msg()
                lane_marker.ns = "lane_line"
                lane_marker.id = 0
                lane_marker.type = Marker.LINE_STRIP  # hoặc TRIANGLE_LIST nếu muốn fill
                lane_marker.action = Marker.ADD
                lane_marker.pose.orientation.w = 1.0
                lane_marker.scale.x = 0.05  # độ dày của line (đơn vị mét)
                lane_marker.color.r = 1.0
                lane_marker.color.g = 0.0
                lane_marker.color.b = 0.5
                lane_marker.color.a = 0.6

                # 🎯 TỪ pixel (x, y) → meter (x_m, y_m)
                meter_per_pixel = 0.005  # <-- cần đo chuẩn lại giá trị này!
                for pt in lane_pts[:, 0, :]:  # (x, y)
                    x_m = (pt[0] - frame.shape[1] / 2) * meter_per_pixel  # lệch trái/phải
                    y_m = (frame.shape[0] - pt[1]) * meter_per_pixel      # gần xa
                    lane_marker.points.append(
                        Point(x=float(x_m), y=float(y_m), z=0.0))

                self.lane_marker_pub.publish(lane_marker)


            # 🎯 Centerline + Steering angle
            if len(lx) >= 5 and len(rx) >= 5:
                center_x = []
                center_y = []
                for y_val in range(0, mask.shape[0], 10):
                    lx_val = int(np.polyval(left_fit, y_val))
                    rx_val = int(np.polyval(right_fit, y_val))
                    cx = (lx_val + rx_val) // 2
                    center_x.append(cx)
                    center_y.append(y_val)
                    cv2.circle(warped, (cx, y_val), 2, (255, 0, 0), -1)

                # Optional: Publish Path
                path_msg = Path()
                path_msg.header.frame_id = "base_link"
                path_msg.header.stamp = self.get_clock().now().to_msg()
                for cx, cy in zip(center_x, center_y):
                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pt = float(self.pixel_to_base_link(cx, cy))
                    # pose.pose.position.x = float(cx)
                    # pose.pose.position.y = float(cy)
                    pose.pose.position = pt
                    pose.pose.orientation.w = 1.0
                    path_msg.poses.append(pose)
                self.centerline_pub.publish(path_msg)

                # Steering angle
                if len(center_x) >= 2:
                    dx = center_x[-1] - center_x[0]
                    dy = center_y[-1] - center_y[0]
                    angle_rad = np.arctan2(-dx, dy)
                    angle_deg = np.degrees(angle_rad)
                    
                    steering_msg = LaneSteering()
                    steering_msg.header.stamp = self.get_clock().now().to_msg()
                    steering_msg.header.frame_id = "base_link"
                    steering_msg.angle_deg = float(angle_deg)
                    steering_msg.angle_rad = float(angle_rad)

                    self.steering_pub.publish(steering_msg)
                    cv2.putText(warped, f"Steering Angle: {angle_deg:.2f} deg", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # cv2.imshow("Lane Detection - Threshold", mask)
            cv2.imshow(self.window_name, frame_copy)
            cv2.imshow("Bird's Eye View", warped)
            cv2.imshow("Lane Detection - Sliding Window", msk)

        key = cv2.waitKey(1)
        if key == ord('r'):
            self.clicked_points.clear()
            self.matrix_ready = False
            self.get_logger().info("🔄 Reset clicked points")
        elif key == ord('q') or key == 27:
            rclpy.shutdown()
        elif key == ord('s'):
            hsv_vals = {"L-H": l_h, "L-S": l_s, "L-V": l_v, "U-H": u_h, "U-S": u_s, "U-V": u_v}
            np.save("trackbar_values.npy", hsv_vals)
            self.get_logger().info("💾 Saved HSV to trackbar_values.npy")

    def pixel_to_base_link(self, px, py):
        #Tọa độ pixel (x_px, y_px) → Tọa độ thực tế (x_m, y_m)

        # px: pixel x (ảnh bird-eye)
        # py: pixel y (ảnh bird-eye)
        meter_per_pixel_x = 0.005  # tùy theo setup của bạn
        meter_per_pixel_y = 0.005

        # Giả sử gốc tọa độ base_link nằm ở giữa cạnh dưới ảnh
        img_center_x = self.output_size[0] // 2
        img_bottom_y = self.output_size[1]

        dx = (px - img_center_x) * meter_per_pixel_x  # sang trái âm, sang phải dương
        dy = (img_bottom_y - py) * meter_per_pixel_y  # càng lên trên thì càng xa

        point = Point()
        point.x = dy  # forward
        point.y = dx  # left-right
        point.z = 0.0
        return point

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
