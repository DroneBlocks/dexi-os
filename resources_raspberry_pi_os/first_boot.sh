#!/bin/bash

# Hostname is set by dexi-set-hostname.service (installed by dexi-networking).
# Hotspot is set up by dexi-hotspot-setup.service (installed by provision.sh).

# Setup DEXI ROS2 launch file to run on boot
# /home/dexi/dexi_ws/src/dexi/scripts/install.bash

# Change dexi directory permissions
chown -R dexi:dexi /home/dexi

# Docker containers are handled by dexi-containers-start.service (setup_docker_containers.sh)

reboot now