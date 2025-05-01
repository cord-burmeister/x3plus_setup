#!/bin/bash

# bash won't source .bashrc from an interactive terminal unless I manually run bash from a terminal:
# $ bash
# or manually source it:
source /home/$USER/.bashrc

workspacename=x3p_ws

# First install required development tools
sudo apt install python3-vcstool python3-colcon-common-extensions git wget -y
sudo apt install libgflags-dev -y


# Then create a new workspace and load the git repositories which are required.
mkdir -p /home/$USER/$workspacename/src
cd /home/$USER/$workspacename/src


wget -O x3plus.repos https://raw.githubusercontent.com/cord-burmeister/x3plus/refs/heads/main/x3plus.repos
vcs import < x3plus.repos

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



