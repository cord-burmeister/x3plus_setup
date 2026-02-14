#!/bin/bash

################################################################################
# Script Name: setup-network.sh
# Description: Installs Samba and NetBIOS name resolution services
#
# Purpose:
#   This script installs Samba and Winbind to enable NetBIOS name resolution
#   on the local network. This allows computers to identify each other using
#   simple names instead of IP addresses within a LAN environment.
#
# Prerequisites:
#   - Ubuntu/Debian-based Linux system
#   - sudo privileges for package installation
#   - Local Area Network (LAN) connection
#
# Usage:
#   bash setup-network.sh
#
# What it does:
#   - Installs samba: File and print sharing service
#   - Installs winbind: Windows NT/2000 domain authentication
#   - Installs libnss-winbind: Name Service Switch plugin for NetBIOS names
#
# Expected Outcome:
#   The system will be able to resolve NetBIOS names (15 character,
#   case-insensitive names) on the local network without needing to know
#   specific IP addresses.
#
# Notes:
#   - NetBIOS names are limited to 15 characters and are not case-sensitive
#   - This is useful for small-scale LAN environments
#   - Additional Samba configuration may be needed in /etc/samba/smb.conf
#
# Author: X3Plus Setup Project
# Version: 1.0
################################################################################


# The design of NetBIOS names allows them to work within a local area network (LAN). They provide a way for computers to identify 
# each other on the same network without understanding complex IP addresses. These names are limited to 15 characters and are not case-sensitive. 
# This makes them easier to remember and use in a small-scale environment.
sudo apt install samba winbind libnss-winbind -y