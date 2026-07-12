FROM osrf/ros:humble-desktop-full

# Instalar Gazebo Fortress y el puente de comunicación
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*
