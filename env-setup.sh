#!/bin/sh

# This file contains only part what should be installed to achieve working environment

sudo apt update
sudo apt install ros-humble-grid-map-msgs
sudo apt install ros-humble-grid-map-core

sudo apt install ros-humble-grid-map-ros

sudo apt install ros-humble-rviz2
sudo apt install ros-humble-grid-map-rviz-plugin

# install cyclone DDS (lower RAM-usage)
sudo apt update
sudo apt install -y ros-humble-rmw-cyclonedds-cpp


sudo apt update
sudo apt install ros-humble-image-transport-plugins
sudo apt install ros-humble-image-view

# install toml
git clone https://github.com/marzer/tomlplusplus.git
cd tomlplusplus
sudo cp -r include/toml++ /usr/local/include/