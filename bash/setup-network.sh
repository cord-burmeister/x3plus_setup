#!/bin/bash


# The design of NetBIOS names allows them to work within a local area network (LAN). They provide a way for computers to identify 
# each other on the same network without understanding complex IP addresses. These names are limited to 15 characters and are not case-sensitive. 
# This makes them easier to remember and use in a small-scale environment.
sudo apt install samba winbind libnss-winbind -y