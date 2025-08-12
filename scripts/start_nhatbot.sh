#!/bin/bash


# Docker image name
DOCKER_IMAGE="nhatbot:latest"


# Allow ROS_DOMAIN_ID to be passed as an argument
ROS_DOMAIN_ID="${1:-7}"

# Environment setup
export DISPLAY=:0
xhost +local:root



log_info()  { echo -e "ℹ️  $1"; }
log_ok()    { echo -e "✅ $1"; }
log_fail()  { echo -e "❌ $1" >&2; return 1; }


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

log_info "Scanning for Arduino Mega device..."
ARDUINO_PORT=$(for dev in /dev/ttyACM* /dev/ttyUSB* /dev/arduino_mega; do
    [[ -e "$dev" ]] || continue
    # Kiểm tra alias hoặc serial đặc trưng của Arduino Mega
    if [[ "$dev" == "/dev/arduino_mega" ]]; then
        echo "$dev"
        break
    fi
    udevadm info -a -n "$dev" | grep -q "55739323837351017271" && echo "$dev" && break
done)

if [ -z "$ARDUINO_PORT" ]; then
    log_fail "Arduino Mega not found"
    exit 1
else
    log_ok "Arduino Mega found at $ARDUINO_PORT"
fi



ESP_USB_PATH_HINT="1-2.4"  # path của ESP, lấy bằng udevadm info

log_info "Scanning for ESP device at USB path: $ESP_USB_PATH_HINT..."
ESP_PORT=""
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [[ -e "$dev" ]] || continue
    udev_path="$(udevadm info -q path -n "$dev" 2>/dev/null || true)"
    if echo "$udev_path" | grep -q "$ESP_USB_PATH_HINT"; then
        ESP_PORT="$dev"
        break
    fi
done

if [[ -z "$ESP_PORT" && -e /dev/esp_device ]]; then
    ESP_PORT="/dev/esp_device"
fi

if [[ -z "$ESP_PORT" ]]; then
    log_fail "ESP device not found at path $ESP_USB_PATH_HINT"
    exit 1
else
    ESP_HOST_REAL="$(readlink -f "$ESP_PORT" 2>/dev/null || echo "$ESP_PORT")"
    log_ok "ESP device found: alias=$ESP_PORT real=$ESP_HOST_REAL"
fi


log_info "Launching Docker container..."
sudo docker run -it \
    --rm \
    --name nhatbot_container \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --env PULSE_SERVER=unix:/run/user/1000/pulse/native \
    --volume $XDG_RUNTIME_DIR/pulse:/run/user/1000/pulse \
    --volume /var/run/dbus:/var/run/dbus \
    --network host \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/video0 \
    --device "$LIDAR_PORT":/dev/rplidar \
    --device "$ZLAC_PORT":/dev/zlac_8015d \
    --device "$ARDUINO_PORT":/dev/arduino_mega \
    --device "$ESP_HOST_REAL":/dev/esp_device \
    --device /dev/bus/usb:/dev/bus/usb \
    --device /dev/snd:/dev/snd \
    --device=/dev/ttyTHS1 \
    --group-add audio \
    --device=/dev/i2c-1 \
    --privileged \
    --volume /home/ninhnt/nhatbot_ws:/home/nhatbot_ws \
    "$DOCKER_IMAGE" \
    /bin/bash
log_ok "Docker container has exited cleanly"