#!/bin/bash

# Stop on build error
#set -e

############################## set up the build processs ##############################
# do this so apt has a dns resolver
mkdir -p /run/systemd/resolve
echo 'nameserver 1.1.1.1' > /run/systemd/resolve/stub-resolv.conf
#######################################################################################

#################################### update the OS ####################################
apt-get update -y &&  apt-get upgrade -y
#######################################################################################

################################ replace boot config ###################################
{
   echo '[all]'
   echo 'kernel=vmlinuz'
   echo 'cmdline=cmdline.txt'
   echo 'initramfs initrd.img followkernel'
   echo ''
   echo '[pi4]'
   echo 'max_framebuffers=2'
   echo 'arm_boost=1'
   echo ''
   echo '[all]'
   echo '# Enable the audio output, I2C and SPI interfaces on the GPIO header. As these'
   echo '# parameters related to the base device-tree they must appear *before* any'
   echo '# other dtoverlay= specification'
   echo 'dtparam=audio=on'
   echo 'dtparam=i2c_arm=on'
   echo 'dtparam=spi=on'
   echo ''
   echo '# Comment out the following line if the edges of the desktop appear outside'
   echo '# the edges of your display'
   echo 'disable_overscan=1'
   echo ''
   echo '# Enable the serial pins'
   echo 'enable_uart=1'
   echo ''
   echo '# Autoload overlays for any recognized cameras or displays that are attached'
   echo '# to the CSI/DSI ports. Please note this is for libcamera support, *not* for'
   echo '# the legacy camera stack'
   echo 'camera_auto_detect=1'
   echo 'display_auto_detect=1'
   echo ''
   echo '# Config settings specific to arm64'
   echo 'arm_64bit=1'
   echo 'dtoverlay=dwc2'
   echo ''
   echo '[cm4]'
   echo '# Enable the USB2 outputs on the IO board (assuming your CM4 is plugged into'
   echo '# such a board)'
   echo 'dtoverlay=dwc2,dr_mode=host'
   echo ''
   echo '[all]'
   echo '# Enable I2C interfaces for additional peripherals'
   echo 'dtparam=i2c_arm=on'
   echo 'dtoverlay=i2c4,pins_6_7   # I2C on GPIO pins 6 and 7'
   echo 'dtoverlay=i2c5,pins_10_11 # I2C on GPIO pins 10 and 11'
   echo ''
   echo '# Set up serial port(s)'
   echo 'enable_uart=1'
   echo 'dtoverlay=uart0'
   echo 'dtoverlay=uart3'
   echo 'dtoverlay=uart4'
   echo ''
   echo '# Set up overlays for ArduCam and IMX519 camera modules'
   echo 'dtoverlay=arducam-pivariety'
   echo 'dtoverlay=imx519'
   echo 'dtoverlay=imx519,cam0'
} > /boot/firmware/config.txt
#######################################################################################

########################### install bootstrapping packages ############################
apt-get install -y \
wget \
curl \
git \
openssh-server \
adduser \
whois \
unzip
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
#groupadd docker
usermod -aG docker dexi
#######################################################################################

#################################### install ROS 2 ####################################
# Setup apt repos
apt install software-properties-common
add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update

# Install ROS2 humble
apt install ros-humble-ros-base ros-dev-tools ros-humble-rosbridge-server ros-humble-topic-tools ros-humble-camera-ros -y
echo "source /opt/ros/humble/setup.bash" >> /home/dexi/.bashrc
echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
rosdep init
rosdep update
#######################################################################################

################################## install neofetch ###################################
apt-get install -y neofetch
echo 'neofetch' >> /home/dexi/.bashrc
#######################################################################################

################################### clone and build dexi repo #########################
mkdir -p /home/dexi/dexi_ws/src
git clone -b feature/raspanion https://github.com/DroneBlocks/dexi.git /home/dexi/dexi_ws/src
cd /home/dexi/dexi_ws/src/dexi
git submodule update --init --remote --recursive
echo "source /home/dexi/dexi_ws/install/setup.bash" >> /home/dexi/.bashrc
echo "source /home/dexi/dexi_ws/install/setup.bash" >> /root/.bashrc

cd /home/dexi/dexi_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select dexi_interfaces
colcon build --packages-select px4_msgs
colcon build --packages-select micro_ros_agent

# So dexi_interfaces are available to the packages below
source /home/dexi/dexi_ws/install/setup.bash

colcon build --packages-select dexi_py
colcon build --packages-select droneblocks
colcon build --packages-select dexi

colcon build --packages-select web_video_server # To make pi camera available on port 8080

colcon build --packages-select apriltag_ros

# Copy camera calibration files into place
mkdir -p /root/.ros/camera_info

# Pi Camera v2.1
cp /home/dexi/dexi_ws/src/dexi/camera_info/main_camera_2.1.yaml /root/.ros/camera_info/imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x480.yaml

# Arducam IMX519 16MP 640x480
cp /home/dexi/dexi_ws/src/dexi/camera_info/arducam_imx519_16mp_640_480.yaml /root/.ros/camera_info/imx519__base_soc_i2c0mux_i2c_0_imx519_1a_640x480.yaml

# Arducam IMX519 16MP 320x240
cp /home/dexi/dexi_ws/src/dexi/camera_info/arducam_imx519_16mp_320_240.yaml /root/.ros/camera_info/imx519__base_soc_i2c0mux_i2c_0_imx519_1a_320x240.yaml

#######################################################################################

################ clone node-red project for use in first_boot.sh ######################
git clone https://github.com/DroneBlocks/node-red-dexi /home/dexi/node-red-dexi
#######################################################################################

#################################### clone ark repo ###################################
git clone https://github.com/DroneBlocks/ark_companion_scripts.git /home/dexi/ark_companion_scripts
cd /home/dexi/ark_companion_scripts
./setup.sh
#######################################################################################

################################### clone wifi repo ###################################
apt install -y iw wireless-tools
git clone https://github.com/Autodrop3d/raspiApWlanScripts.git /home/dexi/wifi_utilities
#######################################################################################

################################### python led packages ###############################
pip3 install rpi_ws281x
pip3 install adafruit-blinka
pip3 install adafruit-circuitpython-neopixel
pip3 install adafruit-circuitpython-led-animation

# Raspanion
pip3 install smbus2
pip3 install pyserial
apt install python3-pigpio

git clone https://github.com/Raspanion/Raspanion/ /home/dexi/raspanion
#######################################################################################

# For some reason this is continuously failing but works fine standalone
# pip3 install requests
# git clone https://github.com/NotGlop/docker-drag /home/dexi/docker-drag
# cd /home/dexi/docker-drag
# python3 docker_pull.py droneblocks/dexi-droneblocks:latest
# python3 docker_pull.py droneblocks/dexi-node-red:latest

mkdir /home/dexi/docker-drag
cd /home/dexi/docker-drag
curl -L -o droneblocks_dexi-droneblocks.tar "https://www.dropbox.com/scl/fi/ag6tml3hpqtg4g0olrkcc/droneblocks_dexi-droneblocks.tar?rlkey=yzr631iv0dv0dgjtzcolbozmg&st=eeaigvva&dl=1"
curl -L -o droneblocks_dexi-node-red.tar "https://www.dropbox.com/scl/fi/a51ndr8s8xgz5swqu0rp8/droneblocks_dexi-node-red.tar?rlkey=2lphmkcbgcwtebws75mh40lbr&st=jmk2s8tc&dl=1"
#######################################################################################

########################## PX4 ROS Node for Navigation ################################
# git clone https://github.com/dbaldwin/PX4-ROS-Node /home/dexi/PX4-ROS-Node
# pip3 install pysm
# pip3 install flask
#######################################################################################

########################## pigpiod for servo control ##################################
cd /home/dexi
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
rm /home/dexi/master.zip
#######################################################################################

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