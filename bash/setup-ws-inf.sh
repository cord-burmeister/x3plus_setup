#!/bin/bash

################################################################################
# Script Name: setup-ws-inf.sh
# Description: Sets up X3Plus infrastructure workspace
#
# Purpose:
#   This script creates and configures the ROS 2 workspace for the X3Plus
#   infrastructure components. It clones the necessary repositories,
#   installs dependencies, and configures the user environment.
#
# Prerequisites:
#   - ROS 2 Humble installed (run setup-humble.sh first)
#   - Internet connection for cloning repositories
#   - sudo privileges for installing dependencies
#
# Usage:
#   bash setup-ws-inf.sh
#
# What it does:
#   1. Sources the user's .bashrc to load ROS environment
#   2. Installs development tools (vcstool, colcon, git, wget, libgflags-dev)
#   3. Creates workspace directory: ~/x3plus_ws/src
#   4. Downloads and imports x3plus_inf repositories
#   5. Initializes and updates rosdep
#   6. Installs all ROS dependencies for the workspace
#   7. Configures .bashrc with:
#      - RCUTILS_COLORIZED_OUTPUT for better logging
#      - Auto-sourcing of workspace setup
#      - Auto-navigation to workspace directory
#
# Expected Outcome:
#   - Workspace created at ~/x3plus_ws with infrastructure packages
#   - Dependencies installed and ready to build with 'colcon build'
#   - .bashrc configured for automatic workspace setup
#
# Directory Structure:
#   ~/x3plus_ws/src/     - Main workspace with infrastructure packages
#
# Notes:
#   - The script is idempotent for .bashrc modifications
#   - You need to build the workspace with 'colcon build' after running this
#   - Source your .bashrc or open a new terminal for changes to take effect
#   - Use this for infrastructure components setup
#
# Author: X3Plus Setup Project
# Version: 1.0
################################################################################

# bash won't source .bashrc from an interactive terminal unless I manually run bash from a terminal:
# $ bash
# or manually source it:
source /home/$USER/.bashrc

workspacename=x3plus_ws

# First install required development tools
sudo apt install python3-vcstool python3-colcon-common-extensions git wget -y
sudo apt install libgflags-dev -y

# Then create a new workspace and load the git repositories which are required.
mkdir -p /home/$USER/$workspacename/src
cd /home/$USER/$workspacename/src


wget -O x3plus_inf.repos https://raw.githubusercontent.com/cord-burmeister/x3plus_inf/refs/heads/main/x3plus_inf.repos
vcs import < x3plus_inf.repos

# Before building the workspace, you need to resolve the package dependencies. 
# You may have all the dependencies already, but best practice is to check for 
# dependencies every time you clone. You wouldn’t want a build to fail after 
# a long wait only to realize that you have missing dependencies.

cd /home/$USER/$workspacename
if [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo rosdep init  was already running.....
else
    sudo rosdep init
fi

rosdep update    
echo rosdep install -r -y --from-path src --rosdistro $ROS_DISTRO 
rosdep install -r -y --from-path src --rosdistro humble

# cd /home/vagrant/$workspacename
# colcon build

source /home/$USER/$workspacename/install/setup.bash

if (grep -q "export RCUTILS_COLORIZED_OUTPUT=1" /home/$USER/.bashrc); then
    echo "RCUTILS_COLORIZED_OUTPUT already set in .bashrc"
else
# Add some help finding errors in the logging 
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/$USER/.bashrc
fi

 # Add sourcing to your shell startup script

if (grep -q "source /home/$USER/$workspacename/install/setup.bash" /home/$USER/.bashrc ) then 
    echo "source /home/$USER/$workspacename/install/setup.bash already set in .bashrc"
else
 echo "source /home/$USER/$workspacename/install/setup.bash" >> /home/$USER/.bashrc
fi


if (grep -q "cd /home/$USER/$workspacename" /home/$USER/.bashrc ) then 
    echo "cd /home/$USER/$workspacename already set in .bashrc"
else
 echo "cd /home/$USER/$workspacename" >> /home/$USER/.bashrc
fi



