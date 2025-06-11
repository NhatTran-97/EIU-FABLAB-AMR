FROM arm64v8/ros:humble
# FROM nhatbot:latest

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home 
# SHELL ["/bin/bash", "-c"]

# Cài đặt curl, gnupg2 và lsb-release
RUN apt-get update && apt-get install -y curl 

# Cập nhật và cài đặt các gói cần thiết
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    nano \
    gedit \
    net-tools \
    usbutils \
    tzdata \
    v4l-utils \
    netcat \
    git \
    curl \
    wget \
    build-essential \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libgl1-mesa-glx \
    libsm6 libxext6 libxrender-dev \
    dbus-x11 \
    libmodbus-dev \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    ros-humble-joy \
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
    ros-humble-nav2-amcl \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-simple-commander \
    ros-humble-robot-localization \
    ros-humble-rqt-tf-tree \
    ros-humble-ros2-control && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Cài đặt các thư viện Python cần thiết
RUN python3 -m pip install --upgrade pip && \
    pip3 install --no-cache-dir --break-system-packages \
    opencv-python \
    pymodbus==2.3.0 \
    asyncio \
    pyserial \
    transforms3d \
    websocket-client

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /home/nhatbot_ws/install/setup.bash" >> /root/.bashrc



# Mở terminal bash khi container chạy
CMD ["/bin/bash"]
