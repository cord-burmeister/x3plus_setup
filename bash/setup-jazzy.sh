#!/bin/bash

################################################################################
# Script Name: setup-jazzy.sh
# Description: Installs ROS 2 Jazzy Jalisco on Ubuntu systems
#
# Purpose:
#   This script automates the installation of ROS 2 Jazzy Jalisco, including
#   the desktop installation (ROS, RViz, demos, tutorials) and development tools.
#   It configures system locales, adds ROS repositories, and installs all
#   necessary packages for ROS 2 development.
#
# Prerequisites:
#   - Ubuntu 24.04 (Noble Numbat) or compatible version
#   - Internet connection for downloading packages
#   - sudo privileges for package installation
#
# Usage:
#   bash setup-jazzy.sh
#
# What it does:
#   1. Configures UTF-8 locale (en_US.UTF-8)
#   2. Enables Ubuntu Universe repository
#   3. Adds ROS 2 GPG key and repository
#   4. Updates system packages
#   5. Installs ros-jazzy-desktop (full desktop installation)
#   6. Installs ros-dev-tools (development tools and compilers)
#
# Expected Outcome:
#   ROS 2 Jazzy will be installed in /opt/ros/jazzy/
#   You can source the environment with: source /opt/ros/jazzy/setup.bash
#
# Notes:
#   - After installation, you need to source the setup file or run
#     setup-jazzy-user.sh to add it to your .bashrc
#   - This script may take 10-30 minutes depending on your internet speed
#
# Author: X3Plus Setup Project
# Version: 1.0
################################################################################

# Make sure you have a locale which supports UTF-8. If you are in a minimal 
# environment (such as a docker container), the locale may be something minimal
#  like POSIX. We test with the following settings. However, it should be fine 
# if you’re using a different UTF-8 supported locale.
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

# First ensure that the Ubuntu Universe repository is enabled.
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# Now add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update your apt repository caches after setting up the repositories.
sudo apt update -y

# ROS 2 packages are built on frequently updated Ubuntu systems. It is always 
# recommended that you ensure your system is up to date before installing new packages.
sudo apt upgrade -y

# Desktop Install (Recommended): ROS, RViz, demos, tutorials.
sudo apt install ros-jazzy-desktop -y

# Development tools: Compilers and other tools to build ROS packages
sudo apt install ros-dev-tools -y

