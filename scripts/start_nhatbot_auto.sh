#!/bin/bash
export DISPLAY=:0
xhost +local:root

if [[ ! -e /dev/rplidar || ! -e /dev/zlac_8015d ]]; then
    echo "❌ Alias /dev/rplidar hoặc /dev/zlac_8015d chưa tồn tại. Hãy kiểm tra udev rules và replug thiết bị."
    exit 1
fi

sudo docker run -it \
    --rm \
    --name nhatbot_container \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=7" \
    --network=host \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/video0 \
    --device=/dev/bus/usb \
    --device /dev/rplidar \
    --device /dev/zlac_8015d \
    --privileged \
    --volume="/home/ninhnt/nhatbot_ws:/home/nhatbot_ws" \
    nhatbot:latest \
    /bin/bash -c "source /home/nhatbot_ws/install/setup.bash && ros2 launch nhatbot_stack nhatbot_bringup.launch.py"


