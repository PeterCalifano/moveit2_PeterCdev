#!/bin/bash
# Script install moveit2 from source. It assumes Ubuntu 24.04 and ROS2 Jazzy Jalisco installed.

# Prepare ROS2 environment
source /opt/ros/jazzy/setup.bash
sudo apt install python3-rosdep

sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade

# Install colcon with mixin
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

sudo apt install python3-vcstool

# Create test workspace
mkdir -p ~/devDir/ws_ros/ws_moveit

# Remove any other installation
sudo apt remove ros-$ROS_DISTRO-moveit*

# Install dependencies
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# Configure colcon worspace
cd ~/devDir/ws_ros/ws_moveit
colcon build --mixin release