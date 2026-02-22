#!/bin/bash

################################################################################
# Script Name: setup-box-development.sh
# Description: Installs essential development and system monitoring tools
#
# Purpose:
#   This script installs commonly used development utilities for system
#   monitoring and file management on the development box.
#
# Prerequisites:
#   - Ubuntu/Debian-based Linux system
#   - sudo privileges for package installation
#
# Usage:
#   bash setup-box-development.sh
#
# What it does:
#   - Installs btop: Modern system resource monitor (CPU, memory, disk, network)
#   - Installs mc (Midnight Commander): Text-based file manager
#
# Expected Outcome:
#   You can run 'btop' for system monitoring or 'mc' for file management
#
# Notes:
#   - btop is a modern alternative to htop/top with better visualization
#   - Midnight Commander provides a dual-pane file manager in the terminal
#
# Author: X3Plus Setup Project
# Version: 1.0
################################################################################

# Install btop for system monitoring
sudo apt install -y btop

# Install Midnight Commander for file management
sudo apt install -y mc 

