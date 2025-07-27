#!/bin/bash

# Stop on build error
#set -e

# Redirect stderr to /dev/null to suppress verbose output, keep only echo statements
exec 2>/dev/null

# Function to run commands quietly
quiet_run() {
    "$@" >/dev/null 2>&1
}

# Logging function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

############################## set up the build processs ##############################
# do this so apt has a dns resolver
mkdir -p /run/systemd/resolve
echo 'nameserver 1.1.1.1' > /run/systemd/resolve/stub-resolv.conf
#######################################################################################

#################################### update the OS ####################################
log "Updating system packages..."
apt-get update -y >/dev/null 2>&1 && apt-get upgrade -y >/dev/null 2>&1
log "System packages updated successfully"
#######################################################################################

# ROS Setup
rm -rf /home/dexi/dexi_ws/*

mkdir -p /home/dexi/dexi_ws/src

cd /home/dexi/dexi_ws
git clone http://github.com/droneblocks/dexi_bringup /home/dexi/dexi_ws/src/dexi_bringup
vcs import --input /home/dexi/dexi_ws/src/dexi_bringup/dexi.repos /home/dexi/dexi_ws/src/
source /home/dexi/ros2_jazzy/install/setup.bash

# Put boot config into place
echo "Writing config.txt contents to /boot/config.txt..."
cat /home/dexi/dexi_ws/src/dexi_bringup/config/cm4/config.txt >> /boot/config.txt

# April tag dependencies
colcon build --packages-select image_geometry
colcon build --packages-select cv_bridge
colcon build --packages-select apriltag
colcon build --packages-select apriltag_msgs
colcon build --packages-select topic_tools_interfaces
colcon build --packages-select compressed_image_transport
colcon build --packages-select compressed_depth_image_transport
apt install -y libtheora-dev
colcon build --packages-select theora_image_transport
colcon build --packages-select zstd_image_transport
colcon build --packages-select image_transport_plugins

apt install -y libcamera-dev
colcon build --packages-select camera_ros

# BEGIN MAVLINK ROUTER
sudo apt install -y meson ninja-build pkg-config gcc g++ systemd
cd /home/dexi
git clone https://github.com/mavlink-router/mavlink-router
cd mavlink-router
git submodule update --init --recursive
meson setup build .
ninja -C build
ninja -C build install
cd /home/dexi
rm -rf mavlink-router
mkdir -p /etc/mavlink-router
cp /home/dexi/dexi_ws/src/dexi_bringup/config/mavlink-router/ark_cm4_main.conf /etc/mavlink-router/main.conf
systemctl enable mavlink-router.service
# END MAVLINK ROUTER






