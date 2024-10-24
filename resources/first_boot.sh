#!/bin/bash

#determine the SSID to configure
FULL_MAC=$(cat /sys/class/net/wlan0/address)
PARTIAL_MAC=$(echo $FULL_MAC | awk -F: '{print $(NF-1)$NF}')
DEXI_SSID="dexi_$PARTIAL_MAC"

# Setup hostname
hostnamectl hostname $DEXI_SSID
# Doing it this way because the underscore gets stripped
echo "127.0.0.1 $(hostnamectl hostname)" >> /etc/hosts

# Setup DEXI ROS2 launch file to run on boot
/home/dexi/dexi_ws/src/dexi/scripts/install.bash

#setup the wifi stuff but use placeholders for first boot
/home/dexi/wifi_utilities/setup_wlan_and_AP_modes.sh -d -s XXXXXXXX -p XXXXXXXX  -a $DEXI_SSID -r droneblocks

# Change dexi directory permissions
chown -R dexi:dexi /home/dexi

# Load DroneBlocks image from provision script since Docker isn't readily available during provisioning
docker load < /home/dexi/docker-drag/droneblocks_dexi-droneblocks.tar
docker load < /home/dexi/docker-drag/droneblocks_dexi-node-red.tar
docker run -d --restart unless-stopped -p 80:80 --name dexi-droneblocks droneblocks/dexi-droneblocks:latest
docker run -d --restart unless-stopped -p 1880:1880 /home/dexi/node-red/flows:/data --name dexi-node-red droneblocks/dexi-node-red:latest
#rm -rf /home/dexi/docker-drag

# Boot faster
systemctl disable systemd-networkd-wait-online.service
systemctl mask systemd-networkd-wait-online.service

reboot now