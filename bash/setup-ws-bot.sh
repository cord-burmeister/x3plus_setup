#!/bin/bash

# bash won't source .bashrc from an interactive terminal unless I manually run bash from a terminal:
# $ bash
# or manually source it:
source /home/$USER/.bashrc

workspacename=m3_ws

# First install required development tools
sudo apt install python3-vcstool python3-colcon-common-extensions git wget -y
sudo apt install libgflags-dev -y

# Then create a folder which contains the driver and hardware dependend components
cd /home/$USER

if [ -e m3plus_driver.yaml ]; then
    rm m3plus_driver.yaml
fi
wget -O m3plus_driver.yaml https://raw.githubusercontent.com/cord-burmeister/x3plus_driver/refs/heads/main/m3plus_driver.yaml
vcs import < m3plus_driver.yaml


# Then create a new workspace and load the git repositories which are required.
mkdir -p /home/$USER/$workspacename/src
cd /home/$USER/$workspacename/src


# wget https://raw.githubusercontent.com/cord-burmeister/m3plus_driver/main/m3plus_driver.yaml
# vcs import < m3plus_driver.yaml

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

# source /home/vagrant/$workspacename/install/setup.bash

# Add some help finding errors in the logging 
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/$USER/.bashrc 

 # Add sourcing to your shell startup script
echo "source /home/$USER/$workspacename/install/setup.bash" >> /home/$USER/.bashrc
echo "cd /home/$USER/$workspacename" >> /home/$USER/.bashrc



