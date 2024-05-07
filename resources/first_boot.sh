#!/bin/bash

#determine the SSID to configure
FULL_MAC=$(cat /sys/class/net/wlan0/address)
PARTIAL_MAC=$(echo $FULL_MAC | awk -F: '{print $(NF-1)$NF}')
DEXI_SSID="dexi_$PARTIAL_MAC"

#setup the wifi stuff but use placeholders for first boot
/home/dexi/wifi_utilities/setup_wlan_and_AP_modes.sh -d -s XXXXXXXX -p XXXXXXXX  -a $DEXI_SSID -r droneblocks

systemctl disable systemd-networkd-wait-online.service
systemctl mask systemd-networkd-wait-online.service

reboot now