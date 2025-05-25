#!/bin/bash

# set -euo pipefail

# Docker image name
DOCKER_IMAGE="nhatbot:latest"


# Allow ROS_DOMAIN_ID to be passed as an argument
ROS_DOMAIN_ID="${1:-7}"

# Environment setup
export DISPLAY=:0
xhost +local:root


log_info()  { echo -e "ℹ️  $1"; }
log_ok()    { echo -e "✅ $1"; }
log_fail()  { echo -e "❌ $1" >&2; exit 1; }



# Check if the Docker image exists
if ! docker image inspect "$DOCKER_IMAGE" > /dev/null 2>&1; then
    log_fail "Docker image '$DOCKER_IMAGE' not found. Please build it first."
fi

# Fixed serial numbers for the devices
ZLAC_SERIAL="AB0PJ5HV"
LIDAR_SERIAL="0001"



log_info "Scanning for ZLAC device..."
ZLAC_PORT=$(for dev in /dev/ttyUSB*; do
    [[ -e "$dev" ]] || continue
    udevadm info -a -n "$dev" | grep -q "$ZLAC_SERIAL" && echo "$dev" && break
done)

if [ -z "$ZLAC_PORT" ]; then
    log_fail "ZLAC device with serial $ZLAC_SERIAL not found"
    exit 1
else
    log_ok "ZLAC found at $ZLAC_PORT"
fi

log_info "Scanning for LiDAR device..."
LIDAR_PORT=$(for dev in /dev/ttyUSB*; do
    [[ -e "$dev" ]] || continue
    udevadm info -a -n "$dev" | grep -q "$LIDAR_SERIAL" && echo "$dev" && break
done)

if [ -z "$LIDAR_PORT" ]; then
    log_fail "LiDAR device with serial $LIDAR_SERIAL not found"
    exit 1
else
    log_ok "LiDAR found at $LIDAR_PORT"
fi


log_info "Launching Docker container..."
sudo docker run -it \
    --rm \
    --name nhatbot_container \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --network host \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/video0 \
    --device "$LIDAR_PORT":/dev/rplidar \
    --device "$ZLAC_PORT":/dev/zlac_8015d \
    --device /dev/bus/usb \
    --privileged \
    --volume /home/ninhnt/nhatbot_ws:/home/nhatbot_ws \
    "$DOCKER_IMAGE" \
    /bin/bash
log_ok "Docker container has exited cleanly"