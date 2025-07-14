#!/bin/bash

# Stop on build error
#set -e

############################## set up the build processs ##############################
# do this so apt has a dns resolver
mkdir -p /run/systemd/resolve
echo 'nameserver 1.1.1.1' > /run/systemd/resolve/stub-resolv.conf
#######################################################################################

#################################### update the OS ####################################
apt-get update -y && apt-get upgrade -y
#######################################################################################

################################ update boot config ###################################

{
   echo ""
   echo '# Dennis Custom Image Build Test'
   echo ""
} >> /boot/config.txt

rm -rf /home/dexi/dexi_ws/*

mkdir -p /home/dexi/dexi_ws/src

cd /home/dexi/dexi_ws

git clone http://github.com/droneblocks/dexi_bringup /home/dexi/dexi_ws/src/dexi_bringup

vcs import --input /home/dexi/dexi_ws/src/dexi_bringup/dexi.repos /home/dexi/dexi_ws/src/

source /home/dexi/ros2_jazzy/install/setup.bash

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
colcon build --packages-select apriltag_ros

# DEXI bringup
colcon build --packages-select dexi_bringup







