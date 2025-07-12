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
} >> /boot/config.txt

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
unzip \
software-properties-common
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
useradd -m -p "$PASS_HASH" -s /bin/bash dexi
usermod -aG sudo dexi
#######################################################################################

###################################  install docker ###################################
curl --output /tmp/get-docker.sh https://get.docker.com
chmod +x /tmp/get-docker.sh
/tmp/get-docker.sh
usermod -aG docker dexi
#######################################################################################

#################################### install ROS 2 ####################################
# Setup apt repos for Raspberry Pi OS (Debian-based)
apt install -y software-properties-common
add-apt-repository universe
apt update

apt install -y neofetch