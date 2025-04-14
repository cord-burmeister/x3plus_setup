#!/bin/bash

# bash won't source .bashrc from an interactive terminal unless I manually run bash from a terminal:
# $ bash
# or manually source it:
source /home/$USER/.bashrc

workspacename=master3_ws

# First install required development tools
sudo apt install python3-vcstool python3-colcon-common-extensions git wget -y
sudo apt install libgflags-dev -y

# Then create a new workspace and load the git repositories which are required.
mkdir -p /home/$USER/$workspacename/src
cd /home/$USER/$workspacename/src

wget https://raw.githubusercontent.com/cord-burmeister/master3_nav/main/master3_nav.yaml
vcs import < master3_nav.yaml
wget https://raw.githubusercontent.com/cord-burmeister/master3_bt3/main/master3_bt3.yaml
vcs import < master3_bt3.yaml
wget https://raw.githubusercontent.com/cord-burmeister/master3_sim/main/master3_sim.yaml
vcs import < master3_sim.yaml

# Before building the workspace, you need to resolve the package dependencies. 
# You may have all the dependencies already, but best practice is to check for 
# dependencies every time you clone. You wouldn’t want a build to fail after 
# a long wait only to realize that you have missing dependencies.

cd /home/$USER/$workspacename
sudo rosdep init
rosdep update    
echo rosdep install -r -y --from-path src --rosdistro $ROS_DISTRO 
rosdep install -r -y --from-path src --rosdistro humble

# cd /home/vagrant/$workspacename
# colcon build

# Cleanup conflicting installed dependencies
# This comes from mixing source and binary packages in the humble harmonic combination
sudo apt autoremove -y
# Reinstall harmonic gazebo, due to conflicting depenency removal.
sudo apt install gz-harmonic -y

# source /home/vagrant/$workspacename/install/setup.bash

# Add some help finding errors in the logging 
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/$USER/.bashrc 
 # Add sourcing to your shell startup script
echo "source /home/$USER/$workspacename/install/setup.bash" >> /home/$USER/.bashrc
echo "cd /home/$USER/$workspacename" >> /home/$USER/.bashrc



