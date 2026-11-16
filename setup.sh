#!/bin/bash

sudo apt install --quiet --assume-yes software-properties-common
sudo add-apt-repository --yes universe

sudo apt update && sudo apt install --quiet --assume-yes curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install --quiet --assume-yes ros-dev-tools

sudo apt update

sudo apt upgrade --quiet --assume-yes

sudo apt install --quiet --assume-yes ros-jazzy-desktop

sudo apt install --quiet --assume-yes ros-jazzy-moveit
sudo apt install --quiet --assume-yes ros-jazzy-hardware-interface
sudo apt install --quiet --assume-yes ros-jazzy-rmw-cyclonedds-cpp
sudo apt install --quiet --assume-yes ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install --quiet --assume-yes ros-jazzy-joint-state-broadcaster
sudo apt install --quiet --assume-yes ros-jazzy-ros-testing
sudo apt install --quiet --assume-yes nlohmann-json3-dev

