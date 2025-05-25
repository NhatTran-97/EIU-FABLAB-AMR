#!/bin/bash

export DISPLAY=:0
xhost +local:root

# Serial cố định
ZLAC_SERIAL="AB0PJ5HV"
LIDAR_SERIAL="0001"

# Tìm thiết bị ZLAC
ZLAC_PORT=$(for dev in /dev/ttyUSB*; do
    [[ -e "$dev" ]] || continue
    udevadm info -a -n "$dev" | grep -q "$ZLAC_SERIAL" && echo "$dev" && break
done)

# Tìm thiết bị LIDAR
LIDAR_PORT=$(for dev in /dev/ttyUSB*; do
    [[ -e "$dev" ]] || continue
    udevadm info -a -n "$dev" | grep -q "$LIDAR_SERIAL" && echo "$dev" && break
done)

# Kiểm tra thiết bị
if [ -z "$ZLAC_PORT" ]; then
    echo "❌ Không tìm thấy thiết bị ZLAC"
    exit 1
else
    echo "✅ ZLAC ở $ZLAC_PORT"
fi

if [ -z "$LIDAR_PORT" ]; then
    echo "❌ Không tìm thấy thiết bị LIDAR"
    exit 1
else
    echo "✅ LiDAR ở $LIDAR_PORT"
fi

# # Chạy Docker ánh xạ đúng tên
# sudo docker run -it \
#     --rm \
#     --name nhatbot_container \
#     --env DISPLAY=$DISPLAY \
#     --env QT_X11_NO_MITSHM=1 \
#     --env ROS_DOMAIN_ID=7 \
#     --network host \
#     --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
#     --device /dev/video0 \
#     --device "$ZLAC_PORT":/dev/zlac_8015d \
#     --device "$LIDAR_PORT":/dev/rplidar \
#     --device /dev/bus/usb \
#     --privileged \
#     --volume /home/ninhnt/nhatbot_ws:/home/nhatbot_ws \
#     nhatbot:latest \
#     /bin/bash
