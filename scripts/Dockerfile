FROM arm64v8/ros:humble


ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home 


RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    bluez \
    nano \
    espeak alsa-utils pulseaudio sox libasound2-plugins mpg123 ffmpeg \
    i2c-tools python3-smbus libi2c-dev \
    gedit \
    net-tools \
    usbutils \
    tzdata \
    v4l-utils \
    netcat \
    git \
    curl \
    wget \
    can-utils libsocketcan2 libsocketcan-dev python3-can \
    build-essential \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libgl1-mesa-glx \
    libsm6 libxext6 libxrender-dev \
    dbus-x11 \
    libmodbus-dev \
    libopencv-dev \
    ros-humble-can-msgs \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    ros-humble-joy \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joy-teleop \
    ros-humble-cv-bridge \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-twist-mux \
    ros-humble-rosbridge-server \
    ros-humble-nav2-bringup \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-example-interfaces \ 
    ros-humble-nav2-amcl \
    ros-humble-usb-cam \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-simple-commander \
    ros-humble-robot-localization \
    ros-humble-rqt-tf-tree \
    ros-humble-ros2-control && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip && \
    pip3 install --no-cache-dir --break-system-packages \
    opencv-python \
    pymodbus==2.3.0 \
    asyncio \
    pyserial \
    pydub \
    sounddevice \
    soundfile \
    smbus2 \
    Jetson.GPIO \
    transforms3d \
    cantools \
    websocket-client

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /home/nhatbot_ws/install/setup.bash" >> /root/.bashrc



CMD ["/bin/bash"]
