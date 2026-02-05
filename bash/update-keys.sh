#!/bin/bash

################################################################################
# Script Name: update-keys.sh
# Description: Updates expired ROS repository GPG keys
#
# Purpose:
#   This script fixes the common issue where ROS repository GPG keys expire,
#   causing apt update to fail with "invalid signature" errors.
#
# Prerequisites:
#   - Ubuntu/Debian-based Linux system
#   - sudo privileges
#   - Internet connection
#
# Usage:
#   bash update-keys.sh
#   sudo apt update  # Run this after the script completes
#
# What it does:
#   1. Downloads the latest ROS GPG key from the official repository
#   2. Updates the ROS repository configuration with the new key
#
# Expected Outcome:
#   The ROS repository key will be updated and 'sudo apt update' will work
#   without signature errors.
#
# When to use:
#   Run this script when you see errors like:
#   "Err:6 http://packages.ros.org/ros2/ubuntu jammy InRelease"
#   "The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654"
#
# Notes:
#   - This script is specifically configured for Ubuntu Jammy (22.04)
#   - Adjust the Ubuntu codename if using a different version
#
# Author: X3Plus Setup Project
# Version: 1.0
################################################################################

# Sometimes the key for the ROS archives will expire
# Then the following error occurs when do a sudo apt update
# Err:6 http://packages.ros.org/ros2/ubuntu jammy InRelease 
# The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>

# To fix that update the keys 

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Then the following command shall be successfull

# sudo apt update
