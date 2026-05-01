#!/bin/bash
# DEXI-OS unified provision script.
#
# Behavior switches on the TARGET env var (cm5 / ark_cm4 / pi5), set by
# raspberry_pi_os.pkr.hcl via:
#   environment_vars = ["TARGET=${var.target}"]

# Stop on build error
#set -e

# Redirect stderr to /dev/null to suppress verbose output, keep only echo statements
exec 2>/dev/null

quiet_run() { "$@" >/dev/null 2>&1; }
log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"; }

if [ -z "$TARGET" ]; then
    log "ERROR: TARGET env var not set (expected cm5 / ark_cm4 / pi5)"
    exit 1
fi
case "$TARGET" in
    cm5|ark_cm4|pi5) ;;
    *) log "ERROR: unsupported TARGET=$TARGET"; exit 1 ;;
esac

log "Provisioning DEXI-OS for target: $TARGET"

# Source shared apt packages
source /tmp/resources/apt_packages.sh

############################## set up the build process ##############################
# do this so apt has a dns resolver
mkdir -p /run/systemd/resolve
echo 'nameserver 1.1.1.1' > /run/systemd/resolve/stub-resolv.conf
#######################################################################################

#################################### update the OS ####################################
log "Updating system packages..."
apt-get update -y >/dev/null 2>&1 && apt-get upgrade -y >/dev/null 2>&1
install_common_packages

# CM4 uses pigpio for direct-PWM servo control via DMA daemon (no I2C servo HAT).
if [ "$TARGET" = "ark_cm4" ]; then
    install_pigpio_packages
    systemctl enable pigpiod
fi

log "System packages updated successfully"
#######################################################################################

#################################### ROS workspace ####################################
rm -rf /home/dexi/dexi_ws/*
mkdir -p /home/dexi/dexi_ws/src

cd /home/dexi/dexi_ws
git clone -b main https://github.com/droneblocks/dexi_bringup /home/dexi/dexi_ws/src/dexi_bringup
vcs import --input /home/dexi/dexi_ws/src/dexi_bringup/dexi.repos /home/dexi/dexi_ws/src/
source /home/dexi/ros2_jazzy/install/setup.bash

#################################### boot config ####################################
# dexi_bringup config dir uses "cm4" rather than the build target name "ark_cm4"
case "$TARGET" in
    ark_cm4) BOOT_CONFIG_DIR="cm4" ;;
    *)       BOOT_CONFIG_DIR="$TARGET" ;;
esac
log "Writing $BOOT_CONFIG_DIR boot config to /boot/config.txt and /boot/cmdline.txt..."
cat /home/dexi/dexi_ws/src/dexi_bringup/config/$BOOT_CONFIG_DIR/config.txt > /boot/config.txt
cat /home/dexi/dexi_ws/src/dexi_bringup/config/$BOOT_CONFIG_DIR/cmdline.txt > /boot/cmdline.txt

########################### enable i2c module #########################################
echo "i2c-dev" >> /etc/modules
#######################################################################################

#################################### build ROS packages ####################################
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

# DEXI LED — Python deps differ per platform LED hardware
case "$TARGET" in
    ark_cm4)
        # CM4 uses Adafruit Blinka stack for ripple/loading effects
        pip install --break-system-packages rpi-ws281x
        pip install --break-system-packages adafruit-blinka
        pip install --break-system-packages adafruit-circuitpython-neopixel
        pip install --break-system-packages adafruit-circuitpython-led-animation
        ;;
    cm5|pi5)
        pip install --break-system-packages pi5neo
        ;;
esac
colcon build --packages-select dexi_led

# CM4 has its own GPIO package (uses pigpio); CM5/Pi5 use dexi_cpp's TCA9555 driver
if [ "$TARGET" = "ark_cm4" ]; then
    colcon build --packages-select dexi_gpio
fi

# lg library (abyz.me.uk) is a runtime dependency of dexi_cpp's I2C/GPIO code.
# Build it for every target so dexi_cpp links cleanly.
cd /home/dexi
wget http://abyz.me.uk/lg/lg.zip
unzip lg.zip
cd lg
make
make install
cd ..
rm -rf lg.zip lg
cd /home/dexi/dexi_ws

# DEXI CPP / Offboard
colcon build --packages-select px4_msgs
colcon build --packages-select dexi_cpp
colcon build --packages-select dexi_offboard

# AprilTag stack
colcon build --packages-select image_geometry
colcon build --packages-select cv_bridge
colcon build --packages-select apriltag
colcon build --packages-select apriltag_msgs
colcon build --packages-select topic_tools_interfaces
colcon build --packages-select topic_tools
colcon build --packages-select compressed_image_transport
colcon build --packages-select compressed_depth_image_transport
colcon build --packages-select theora_image_transport
colcon build --packages-select zstd_image_transport
colcon build --packages-select image_transport_plugins

# Strip the camera_ros exec_depend from apriltag_ros so it builds standalone.
# Idempotent — does nothing if camera_ros isn't referenced. Required for Pi 5
# (no camera_ros) and harmless on CM4/CM5 where camera_ros is also installed.
sed -i '/<exec_depend>camera_ros<\/exec_depend>/d' /home/dexi/dexi_ws/src/apriltag_ros/package.xml
colcon build --packages-select apriltag_ros

# Camera packages — CM4/CM5 use camera_ros (libcamera, CSI), Pi 5 uses dexi_camera (UVC)
if [ "$TARGET" != "pi5" ]; then
    install_camera_packages
    colcon build --packages-select camera_ros
fi
# All targets build dexi_camera (provides camera calibration files; Pi 5 also uses it as the camera node)
colcon build --packages-select dexi_camera

# DEXI yolo
pip install --break-system-packages onnxruntime
colcon build --packages-select dexi_yolo

# DEXI color detection (build for all targets — was previously missing on pi5)
colcon build --packages-select dexi_color_detection

# DEXI bringup (the launch package and start.bash)
colcon build --packages-select dexi_bringup

# DEXI AprilTag corridor / precision-landing nodes (apriltag_odometry, tag_hop, precision_landing)
colcon build --packages-select dexi_apriltag

# Install DEXI service
cd /home/dexi/dexi_ws/src/dexi_bringup/scripts
./install.bash

#################################### MAVLink router ####################################
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
case "$TARGET" in
    ark_cm4) MAVLINK_ROUTER_CONF="ark_cm4_main.conf" ;;
    *)       MAVLINK_ROUTER_CONF="main.conf" ;;
esac
cp /home/dexi/dexi_ws/src/dexi_bringup/config/mavlink-router/$MAVLINK_ROUTER_CONF /etc/mavlink-router/main.conf
systemctl enable mavlink-router.service

mkdir -p /etc/systemd/system/mavlink-router.service.d
cat > /etc/systemd/system/mavlink-router.service.d/override.conf << 'EOF'
[Service]
ExecStart=
ExecStart=/usr/bin/mavlink-routerd --syslog
Restart=on-failure
RestartSec=5
EOF
########################################################################################

#################################### ARK companion + PX4 firmware ####################################
# Pi 5 has no flight controller attached, skip both
if [ "$TARGET" = "cm5" ] || [ "$TARGET" = "ark_cm4" ]; then
    cd /home/dexi
    git clone https://github.com/DroneBlocks/ark_companion_scripts.git /home/dexi/ark_companion_scripts
    cd /home/dexi/ark_companion_scripts
    log "Installing ark_companion scripts to /usr/bin"
    for file in "pi/scripts/"*; do
        cp $file /usr/bin
    done

    case "$TARGET" in
        cm5)     PX4_FIRMWARE="ark_pi6x_default_v1.16.1.px4" ;;
        ark_cm4) PX4_FIRMWARE="ark_pi6x_default_v1.16.1-flow-fix.px4" ;;
    esac
    cp /tmp/resources/$PX4_FIRMWARE /home/dexi/
    log "Staged PX4 firmware: $PX4_FIRMWARE"
fi
######################################################################################################

#################################### DEXI networking ####################################
log "Setting up DEXI networking..."
cd /tmp
git clone https://github.com/DroneBlocks/dexi-networking.git
cd dexi-networking
./install.sh

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

cat > /usr/local/bin/dexi-hotspot-setup.sh << 'EOF'
#!/bin/bash
sleep 10
PARTIAL_MAC=$(cat /sys/class/net/wlan0/address | awk -F: '{print $(NF-1)$NF}')
DEXI_SSID="dexi_$PARTIAL_MAC"
/usr/local/bin/dexi/create_hotspot.sh "$DEXI_SSID" "droneblocks"
systemctl disable dexi-hotspot-setup.service
EOF

chmod +x /usr/local/bin/dexi-hotspot-setup.sh
systemctl enable dexi-hotspot-setup.service

log "DEXI networking installed - hotspot will be created on first boot"

cd /
rm -rf /tmp/dexi-networking
##########################################################################################

#################################### Docker containers ####################################
log "Running Docker containers setup script..."
if [ -f "/tmp/resources/setup_docker_containers.sh" ]; then
    chmod +x /tmp/resources/setup_docker_containers.sh
    /tmp/resources/setup_docker_containers.sh
    log "Docker setup script completed"
else
    log "ERROR: Docker setup script not found at /tmp/resources/setup_docker_containers.sh"
fi
###########################################################################################

#################################### code-server ####################################
log "Running code-server setup script..."
if [ -f "/tmp/resources/setup_code_server.sh" ]; then
    chmod +x /tmp/resources/setup_code_server.sh
    /tmp/resources/setup_code_server.sh
    log "Code-server setup script completed"
else
    log "ERROR: Code-server setup script not found at /tmp/resources/setup_code_server.sh"
fi
######################################################################################

# Embed build version for traceability (readable on device via `cat /etc/dexi-version`)
if [ -f /tmp/resources/build_version ]; then
    cp /tmp/resources/build_version /etc/dexi-version
    log "Embedded version: $(cat /etc/dexi-version)"
fi

chown -R dexi:dexi /home/dexi
log "Provisioning complete for target: $TARGET"
