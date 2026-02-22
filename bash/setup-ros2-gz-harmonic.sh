#!/bin/bash

################################################################################
# Script Name: setup-ros2-gz-harmonic.sh
# Description: Installs Gazebo Harmonic simulator
#
# Purpose:
#   This script installs Gazebo Harmonic, the latest generation of the Gazebo
#   robot simulator. It adds the necessary repositories and configures the
#   environment for use with ROS 2.
#
# Prerequisites:
#   - Ubuntu Linux system
#   - sudo privileges for package installation
#   - Internet connection
#
# Usage:
#   bash setup-ros2-gz-harmonic.sh
#
# What it does:
#   1. Installs required dependencies (lsb-release, wget, gnupg)
#   2. Adds Gazebo GPG key and repository
#   3. Updates and upgrades system packages
#   4. Installs gz-harmonic package
#   5. Sets GZ_VERSION environment variable to 'harmonic' in .bashrc
#
# Expected Outcome:
#   Gazebo Harmonic will be installed and the environment variable GZ_VERSION
#   will be set to 'harmonic' for all new terminal sessions.
#
# Notes:
#   - GZ_VERSION variable is added to /home/vagrant/.bashrc
#   - You may need to adjust the user path if not using vagrant user
#   - Gazebo Harmonic is compatible with ROS 2 Humble and later
#
# Author: X3Plus Setup Project
# Version: 1.0
################################################################################

sudo apt-get update
sudo apt-get install lsb-release wget gnupg
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install gz-harmonic -y

 # Add sourcing to your shell startup script
echo "export GZ_VERSION=harmonic" >> /home/vagrant/.bashrc