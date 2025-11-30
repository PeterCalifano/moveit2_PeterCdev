#!/bin/bash
# Script install moveit2 from source. It assumes Ubuntu 24.04 and ROS2 Jazzy Jalisco installed.
# Reference documentation: https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html

moveit2_install_dir=~/devDir/ws_ros/ws_moveit

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
mkdir -p $moveit2_install_dir

# Clone moveit2-tutorials
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos

# Remove any other installation
sudo apt remove ros-$ROS_DISTRO-moveit*

# Install dependencies
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# Configure colcon worspace
cd $moveit2_install_dir
colcon build --mixin release

# NOTE on the command above, from doc:
# Some of the packages built with this command require up to 16Gb of RAM to build. By default, colcon tries to build as many packages as possible at the same time. If you are low on computer memory, or if the build is generally having trouble completing on your computer, you can try appending --executor sequential to the colcon command above to build only one package at a time, or --parallel-workers <X> to limit the number of simultaneous builds. For even more limited machines, you can try running MAKEFLAGS="-j4 -l1" colcon build --executor sequential.

# Source colcon workspace for moveit2
source $moveit2_install_dir/install/setup.bash

# To add to bashrc:
#echo 'source $moveit2_install_dir/install/setup.bash' >> ~/.bashrc
