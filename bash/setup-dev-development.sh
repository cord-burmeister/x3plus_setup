#!/bin/bash

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
