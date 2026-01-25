
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh


# Add sourcing to your shell startup script
if (grep -q "source /opt/ros/jazzy/setup.bash" /home/$USER/.bashrc ) then 
    echo "source /opt/ros/jazzy/setup.bash already set in .bashrc"
else
 echo "source /opt/ros/jazzy/setup.bash" >> /home/$USER/.bashrc
fi
