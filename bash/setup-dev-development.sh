#!/bin/bash

################################################################################
# Script Name: setup-dev-development.sh
# Description: Installs Visual Studio Code via Microsoft repository
#
# Purpose:
#   This script installs Visual Studio Code editor using the official Microsoft
#   repository. This method is preferred over Snap to avoid authentication
#   callback issues.
#
# Prerequisites:
#   - Ubuntu/Debian-based Linux system
#   - sudo privileges for package installation
#   - Internet connection
#
# Usage:
#   bash setup-dev-development.sh
#
# What it does:
#   1. Downloads and adds Microsoft GPG key
#   2. Adds VS Code repository to system sources
#   3. Updates package cache
#   4. Installs Visual Studio Code
#
# Expected Outcome:
#   Visual Studio Code will be installed and accessible via 'code' command
#
# Notes:
#   - The Snap version has known issues with authentication callbacks
#   - Browser login sometimes fails with the Snap version
#   - This Microsoft repository installation avoids those issues
#
# Author: X3Plus Setup Project
# Version: 1.0
################################################################################

# The Snap version has had issues where:
#   The authentication callback fails
# 
# The sign‑in button doesn’t appear
# Browser login cannot return to VS Code
# This is a known limitation.
# sudo snap install --classic code

# Install via the Microsoft repository
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | sudo gpg --dearmor -o /usr/share/keyrings/microsoft.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update
sudo apt install code
