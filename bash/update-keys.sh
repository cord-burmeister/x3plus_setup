#!/bin/bash

# Sometimes the key for the ROS archives will expire
# Then the following error occurs when do a sudo apt update
# Err:6 http://packages.ros.org/ros2/ubuntu jammy InRelease 
# The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>

# To fix that update the keys 

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Then the following command shall be successfull

# sudo apt update
