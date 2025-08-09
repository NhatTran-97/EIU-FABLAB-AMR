# EIU-FABLAB-AMR: Autonomous Mobile Robot Project

## Overview
# EIU-FABLAB-AMR: Autonomous Mobile Robot and Robot Manipulation Project

## Overview

This project focuses on the development of an **Autonomous Mobile Robot (AMR)** and a **Robot Manipulation System (robot arm)** for research and practical applications in the field of robotics. The project is organized into multiple packages, each responsible for different aspects of the robot's functionality, including mobility, manipulation, control, perception, navigation, and safety.

The **Autonomous Mobile Robot (AMR)** is designed to move and navigate autonomously in various environments, while the **Robot Manipulation System** is built around the **Dobot Magician arm** to perform tasks like object manipulation and interaction.

### Features:
- **Autonomous Mobile Robot (AMR)**:
  - **Differential Drive**: A package that implements differential drive control for precise movement and maneuvering.
  - **Navigation**: Provides autonomous navigation capabilities using sensors and algorithms to create a map and plan paths.
  - **Perception**: Includes vision-based and sensor-based perception systems to understand the robot's environment.
  - **Safety**: Implements safety protocols to ensure secure operations and emergency stopping mechanisms.
  
- **Robot Manipulation**:
  - **Robot Arm Control (Dobot Magician)**: Control the **Dobot Magician robotic arm** for various tasks such as pick-and-place, object manipulation, and more.
  - **End-Effector Integration**: Manage tools attached to the robot arm for specific tasks (grippers, suction cups, etc.).
  - **Precision Movement**: Algorithms for precise and repeatable movements required for tasks like assembly or manipulation.

- **ROS2 Integration**: The project utilizes ROS2 for communication between various components and packages.

### Packages in the Project:
- **nhatbot_controller**: Controls the robot's motion and response to commands.
- **nhatbot_navigation**: Handles the robot's navigation and path planning.
- **nhatbot_stack**: The core stack managing the robot's functionalities.
- **nhatbot_safety**: Ensures the robot operates safely in various environments.
- **nhatbot_twist_teleop**: Provides a teleoperation interface for manual control.
- **nhatbot_dobot_magician**: A specialized package for controlling the **Dobot Magician robotic arm** for manipulation tasks.
- **nhatbot_ros2_basic**: Provides basic integration for ROS2 functionalities and services.

### Technologies Used:
- **Python & C++**: For algorithm implementation and system control.
- **ROS2**: The Robot Operating System (ROS) framework to facilitate communication and control.
- **CMake**: For building and managing the robot's software packages.
- **Dobot Magician SDK**: For controlling the robotic arm and integrating the manipulation system.

## Installation

To get started with the robot system, clone the repository and install the required dependencies.

```bash
# Clone the repository
git clone https://github.com/NhatTran-97/EIU-FABLAB-AMR.git

# Install dependencies
cd EIU-FABLAB-AMR
pip install -r requirements.txt


