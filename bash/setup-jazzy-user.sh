#!/bin/bash

################################################################################
# Script Name: setup-jazzy-user.sh
# Description: Configures user environment for ROS 2 Jazzy
#
# Purpose:
#   This script adds the ROS 2 Jazzy setup script to the user's .bashrc file,
#   ensuring that the ROS 2 environment is automatically sourced when a new
#   terminal session is started.
#
# Prerequisites:
#   - ROS 2 Jazzy must be installed (run setup-jazzy.sh first)
#   - Bash shell environment
#
# Usage:
#   bash setup-jazzy-user.sh
#
# What it does:
#   - Checks if the ROS 2 Jazzy source command is already in .bashrc
#   - If not present, adds: source /opt/ros/jazzy/setup.bash
#
# Expected Outcome:
#   The ROS 2 Jazzy environment will be automatically available in all new
#   terminal sessions without manual sourcing.
#
# Notes:
#   - This only affects new terminal sessions; reload your current session
#     with: source ~/.bashrc
#   - The script is idempotent (safe to run multiple times)
#
# Author: X3Plus Setup Project
# Version: 1.0
################################################################################


# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh


# Add sourcing to your shell startup script
if (grep -q "source /opt/ros/jazzy/setup.bash" /home/$USER/.bashrc ) then 
    echo "source /opt/ros/jazzy/setup.bash already set in .bashrc"
else
 echo "source /opt/ros/jazzy/setup.bash" >> /home/$USER/.bashrc
fi
