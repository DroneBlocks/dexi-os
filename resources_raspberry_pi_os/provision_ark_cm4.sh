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
cat /home/dexi/dexi_ws/src/dexi_bringup/config/cm4/config.txt > /boot/config.txt

########################### enable i2c module #########################################
# Add i2c-dev module to /etc/modules for automatic loading on boot
echo "i2c-dev" >> /etc/modules
#######################################################################################

# Rosbridge
colcon build --packages-select rosbridge_test_msgs
colcon build --packages-select rosbridge_library
colcon build --packages-select rosapi_msgs
colcon build --packages-select rosapi
colcon build --packages-select rosbridge_msgs
colcon build --packages-select rosbridge_server

# Micro ROS agent
colcon build --packages-select micro_ros_msgs
colcon build --packages-select micro_ros_agent

# DEXI interfaces
colcon build --packages-select dexi_interfaces

# DEXI LED
pip install --break-system-packages pi5neo
colcon build --packages-select dexi_led

# Dependencies for DEXI CPP
cd /home/dexi
wget http://abyz.me.uk/lg/lg.zip
unzip lg.zip
cd lg
make
make install
cd ..
rm -rf lg.zip
rm -rf lg
cd /home/dexi/dexi_ws

# DEXI CPP
colcon build --packages-select px4_msgs
colcon build --packages-select dexi_cpp

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

# DEXI camera
# Right now this is the CSI camera for CM4
apt install -y libcamera-dev
colcon build --packages-select camera_ros
# Build dexi_camera so we have the calibration file accessible to the camera_ros node
colcon build --packages-select dexi_camera

colcon build --packages-select apriltag_ros

# DEXI bringup
colcon build --packages-select dexi_bringup

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
cp /home/dexi/dexi_ws/src/dexi_bringup/config/mavlink-router/main.conf /etc/mavlink-router/main.conf
systemctl enable mavlink-router.service
# END MAVLINK ROUTER

################################ DEXI NETWORKING ################################
log "Setting up DEXI networking..."

# Clone and install dexi-networking
cd /tmp
git clone https://github.com/DroneBlocks/dexi-networking.git
cd dexi-networking
./install.sh

# Create a service that will create the hotspot on first boot with actual MAC
cat > /etc/systemd/system/dexi-hotspot-setup.service << 'EOF'
[Unit]
Description=DEXI Hotspot Setup
After=network.target
Wants=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/dexi-hotspot-setup.sh
Restart=no

[Install]
WantedBy=multi-user.target
EOF

# Create the setup script that runs on first boot
cat > /usr/local/bin/dexi-hotspot-setup.sh << 'EOF'
#!/bin/bash
# Wait for wlan0 to be available
sleep 10

# Get the actual device MAC address
PARTIAL_MAC=$(cat /sys/class/net/wlan0/address | awk -F: '{print $(NF-1)$NF}')
DEXI_SSID="dexi_$PARTIAL_MAC"

# Create the hotspot with the real MAC address
/usr/local/bin/dexi/create_hotspot.sh "$DEXI_SSID" "droneblocks"

# Disable this service so it only runs once
systemctl disable dexi-hotspot-setup.service
EOF

chmod +x /usr/local/bin/dexi-hotspot-setup.sh
systemctl enable dexi-hotspot-setup.service

log "DEXI networking installed - hotspot will be created on first boot"
log "Users can connect with password: droneblocks"
log "Users can configure their WiFi with: sudo dexi-wifi 'NetworkName' 'password'"

# Clean up temp directory
cd /
rm -rf /tmp/dexi-networking
#################################################################################

chown -R dexi:dexi /home/dexi
