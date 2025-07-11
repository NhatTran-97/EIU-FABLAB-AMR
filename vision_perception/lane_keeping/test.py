import cv2

# Mở webcam (0 là webcam mặc định; thử 1, 2 nếu không hoạt động)
cap = cv2.VideoCapture(0)

# Kiểm tra xem mở được chưa
if not cap.isOpened():
    print("❌ Không thể mở webcam")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Không đọc được frame")
        break

    # Hiển thị ảnh từ webcam
    cv2.imshow("Webcam Feed", frame)

    # Bấm 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng tài nguyên
cap.release()
cv2.destroyAllWindows()
