#!/bin/bash


# Add sourcing to your shell startup script
if (grep -q "source /opt/ros/humble/setup.bash" /home/$USER/.bashrc ) then 
    echo "source /opt/ros/humble/setup.bash already set in .bashrc"
else
 echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
fi

