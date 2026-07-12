FROM osrf/ros:jazzy-desktop-full

# Instalar Gazebo Harmonic y el puente de comunicación
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*
