"""
Chạy robot một quãng đường/ góc cố định.
Ghi lại dữ liệu odometry ước lượng (/odom) và dữ liệu “thật” (ví dụ đo bằng thước, marker, motion
capture hoặc ít nhất so với quỹ đạo mong muốn).
Tính sai số = giá trị ước lượng – giá trị thật.
Covariance = trung bình bình phương sai số (variance).

Thực nghiệm đơn giản: cho robot chạy thẳng 1 m và quay 90° nhiều lần, đo sai số bằng thước, nhập vào script → sẽ ra số liệu “thật” cho YAML.

"""






import numpy as np
import pandas as pd

# Giả sử bạn đã log dữ liệu vào CSV với các cột:
# time, x_odom, y_odom, yaw_odom, x_true, y_true, yaw_true
# x_true, y_true có thể là dữ liệu đo bằng thước hoặc ground truth từ motion capture.

# Đọc dữ liệu
df = pd.read_csv("odom_log.csv")

# Tính sai số
df["err_x"] = df["x_odom"] - df["x_true"]
df["err_y"] = df["y_odom"] - df["y_true"]
df["err_yaw"] = df["yaw_odom"] - df["yaw_true"]

# Tính phương sai (variance = covariance đường chéo)
var_x = np.var(df["err_x"])
var_y = np.var(df["err_y"])
var_yaw = np.var(df["err_yaw"])

print("Variance x:", var_x)
print("Variance y:", var_y)
print("Variance yaw:", var_yaw)

# Xuất ra theo format ROS2 YAML
pose_covariance_diagonal = [var_x, var_y, 0.1, 0.1, 0.1, var_yaw]
print("pose_covariance_diagonal:", pose_covariance_diagonal)
