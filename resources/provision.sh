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
   echo '# Disable SPI for now because it renders /dev/ttyAMA2 useless, which is where we get serial telem for ROS2'
   echo 'dtparam=spi=off'
   echo ""
   echo '# Enable CM4 external antenna'
   echo 'dtparam=ant2'
   echo ""
} >> /boot/firmware/config.txt

#######################################################################################

########################### install bootstrapping packages ############################
apt-get install -y \
wget \
curl \
git \
openssh-server \
adduser \
whois \
python3-pigpio \
unzip
#######################################################################################

################################# install code-server #################################
# Download and install code-server
curl -fsSL https://code-server.dev/install.sh | sh

# Create config directory for root user
mkdir -p /root/.config/code-server

# Create the config.yaml file
cat > /root/.config/code-server/config.yaml << EOF
bind-addr: 0.0.0.0:9999
auth: password
password: droneblocks
cert: false
user-data-dir: /root/.local/share/code-server
extensions-dir: /root/.local/share/code-server/extensions
EOF

# Set dark theme as default
mkdir -p /root/.local/share/code-server/User
cat > /root/.local/share/code-server/User/settings.json << EOF
{
    "workbench.colorTheme": "Default Dark+",
    "workbench.startupEditor": "none"
}
EOF

# Create systemd service file
cat > /etc/systemd/system/code-server.service << EOF
[Unit]
Description=code-server
After=network.target

[Service]
Type=simple
User=root
Environment=HOME=/root
ExecStart=/usr/bin/code-server --config /root/.config/code-server/config.yaml /home/dexi/dexi_ws
Restart=always

[Install]
WantedBy=multi-user.target
EOF

# Enable the service
systemctl enable code-server
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
apt install ros-humble-ros-base ros-dev-tools ros-humble-rosbridge-server ros-humble-topic-tools ros-humble-camera-ros ros-humble-compressed-image-transport -y
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
git clone -b develop https://github.com/DroneBlocks/dexi.git /home/dexi/dexi_ws/src
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
#######################################################################################

# For some reason this is continuously failing but works fine standalone
# pip3 install requests
# git clone https://github.com/NotGlop/docker-drag /home/dexi/docker-drag
# cd /home/dexi/docker-drag
# python3 docker_pull.py droneblocks/dexi-droneblocks:latest
# python3 docker_pull.py droneblocks/dexi-node-red:latest

mkdir /home/dexi/docker-drag
cd /home/dexi/docker-drag
curl -L -o droneblocks_dexi-droneblocks.tar "https://www.dropbox.com/scl/fi/wecfd8im6vtwm61fr39xg/droneblocks_dexi-droneblocks_0.09.tar?rlkey=1uw28pmc98rbdz8jmr3vco3jv&st=jhfs7xwz&dl=1"
curl -L -o droneblocks_dexi-node-red.tar "https://www.dropbox.com/scl/fi/7mdwkra1lwg8ghea3f241/droneblocks_dexi-node-red.tar?rlkey=4ww0oupfz0k90mud8dnra7l7b&st=m95m0le5&dl=1"
#######################################################################################

########################## PX4 ROS Node for Navigation ################################
pip3 install pysm
#######################################################################################

########################## pigpiod for servo control ##################################
cd /home/dexi
wget -O pigpio-master.zip https://github.com/joan2937/pigpio/archive/master.zip
unzip pigpio-master.zip
cd pigpio-master
make
sudo make install
rm /home/dexi/pigpio-master.zip
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