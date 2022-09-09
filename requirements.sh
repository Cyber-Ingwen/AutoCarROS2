#!/bin/bash

sudo apt update
sudo apt install python3-pip
sudo apt install -y \
    python3-argcomplete \
    ros-{ROS_DISTRO}-joint-state-publisher \
    ros-{ROS_DISTRO}-vision-msgs \
    ros-{ROS_DISTRO}-gazebo-ros-pkgs \
    ros-{ROS_DISTRO}-libg2o \
    ros-{ROS_DISTRO}-velodyne*

pip install --upgrade pip
pip install -r requirements.txt
