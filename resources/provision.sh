#!/bin/bash

set -e

############################## set up the build processs ##############################
# do this so apt has a dns resolver
mkdir -p /run/systemd/resolve
echo 'nameserver 1.1.1.1' > /run/systemd/resolve/stub-resolv.conf
# echo "packer-runner" > /etc/hostname
# echo "127.0.1.1 packer-runner" >> /etc/hosts
# cat /etc/hosts
#######################################################################################

#################################### update the OS ####################################
DEBIAN_FRONTEND=noninteractive apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
#######################################################################################

################################ update boot config ###################################

{
   echo ""
   echo '# General Purpose UART External'
   echo 'dtoverlay=uart3,ctsrts'
   echo ""
   echo '# Telem2 to FC'
   echo 'dtoverlay=uart4,ctsrts'
   echo ""
   echo '# Drive USB Hub nRESET high'
   echo 'gpio=26=op,dh'
   echo ""
   echo '# Drive FMU_RST_REQ low'
   echo 'gpio=25=op,dl'
   echo ""
   echo '# Drive USB_OTG_FS_VBUS high to enable FC USB'
   echo 'gpio=27=op,dh'
   echo ""
   echo '# Disable Gigabit Ethernet, it can only do 100Mbps'
   echo 'dtoverlay=cm4-disable-gigabit-ethernet'
   echo ""
   echo 'dtoverlay=imx219,cam0'
   echo 'dtoverlay=imx219,cam1'
   echo ""
   echo 'dtoverlay=gpio-fan,gpiopin=19'
   echo ""
   echo 'Disable SPI for now because it renders /dev/ttyAMA2 useless, which is where we get serial telem for ROS2'
   echo 'dtparam=spi=off'
   echo ""
} >> /boot/firmware/config.txt

#######################################################################################

########################### install bootstrapping packages ############################
DEBIAN_FRONTEND=noninteractive apt-get install -y \
wget \
curl \
git \
openssh-server \
adduser \
whois \
cgroupfs-mount
#######################################################################################

################################## create dexi user ###################################
DEXI_PASS=dexi
PASS_HASH=$(mkpasswd -m sha-512 $DEXI_PASS)
# adduser -D dexi --comment "" --disabled-password
useradd -m -p "$PASS_HASH" -s /bin/bash dexi
usermod -aG sudo dexi
#######################################################################################

###################################  install docker ###################################
curl --output /tmp/get-docker.sh https://get.docker.com
chmod +x /tmp/get-docker.sh
/tmp/get-docker.sh
# groupadd docker
usermod -aG docker dexi

# mkdir -p /sys/fs/cgroup
# mount -t cgroup -o devices cgroup /sys/fs/cgroup

dockerd --debug # -H unix:///var/run/docker.sock

echo "GOT PAST DOCKERD"

docker pull ubuntu:22.04
#######################################################################################

#################################### install ROS 2 ####################################
# Setup apt repos
DEBIAN_FRONTEND=noninteractive apt install software-properties-common
add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null
DEBIAN_FRONTEND=noninteractive apt update

# Install ROS2 humble
DEBIAN_FRONTEND=noninteractive apt install ros-humble-ros-base ros-dev-tools ros-humble-rosbridge-server -y
echo "source /opt/ros/humble/setup.bash" >> /home/dexi/.bashrc
rosdep init
#######################################################################################

################################## install neofetch ###################################
DEBIAN_FRONTEND=noninteractive apt-get install -y neofetch
echo 'neofetch' >> /home/dexi/.bashrc
#######################################################################################

################################### clone and build dexi repo #########################
mkdir -p /home/dexi/dexi_ws/src
git clone -b develop https://github.com/DroneBlocks/dexi.git /home/dexi/dexi_ws/src
git clone https://github.com/micro-ROS/micro_ros_msgs.git /home/dexi/dexi_ws/src/micro_ros_msgs
cd /home/dexi/dexi_ws/src/dexi
git submodule update --init --remote --recursive
echo "source /home/dexi/dexi_ws/install/setup.bash" >> /home/dexi/.bashrc

cd /home/dexi/dexi_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dexi_msgs
colcon build --packages-select led_msgs
colcon build --packages-select px4_msgs
colcon build --packages-select micro_ros_msgs
colcon build --packages-select micro_ros_agent
colcon build --packages-select dexi_py
#######################################################################################

################################### python led packages ###############################
DEBIAN_FRONTEND=noninteractive apt-get install -y python3 python3-pip
pip3 install rpi_ws281x
pip3 install adafruit-blinka
pip3 install adafruit-circuitpython-neopixel
pip3 install adafruit-circuitpython-led-animation
#######################################################################################

#################################### clone ark repo ###################################
git clone https://github.com/DroneBlocks/ark_companion_scripts.git /home/dexi/ark_companion_scripts
cd /home/dexi/ark_companion_scripts
./setup.sh
#######################################################################################

################################### clone wifi repo ###################################
DEBIAN_FRONTEND=noninteractive apt install -y iw wireless-tools
git clone https://github.com/Autodrop3d/raspiApWlanScripts.git /home/dexi/wifi_utilities
#######################################################################################

chown -R dexi:dexi /home/dexi

############################### provision runonce daemon ##############################
# creates a job that only runs once (AKA on first boot)
# we'll use this to provision the wifi stuff once the SD card is put into the PI

# create the dir structure
mkdir -p /etc/local/runonce.d/ran
cp /tmp/resources/runonce /usr/local/bin/runonce
chmod +x /usr/local/bin/runonce

# add the cron job
croncmd="/usr/local/bin/runonce"
cronjob="@reboot $croncmd"
( crontab -l | grep -v -F "$croncmd" ; echo "$cronjob" ) | crontab -

# add the first_boot script
mv /tmp/resources/first_boot.sh /etc/local/runonce.d/first_boot.sh
chmod +x /etc/local/runonce.d/first_boot.sh
#######################################################################################