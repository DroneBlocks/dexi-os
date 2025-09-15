# DEXI OS Image Builder

Custom Raspberry Pi OS images for DEXI drone systems with pre-configured ROS2, networking, and development tools.

## Supported Platforms

### Raspberry Pi Compute Module 4 (CM4)
- **Build Script**: `build_raspberry_pi_os_ark_cm4.sh`
- **Packer Config**: `raspberry_pi_os_ark_cm4.pkr.hcl`
- **Provision Script**: `resources_raspberry_pi_os/provision_ark_cm4.sh`
- **Output Image**: `dexi_raspberry_pi_os_ark_cm4.img`

### Raspberry Pi 5
- **Build Script**: `build_raspberry_pi_os_pi5.sh`
- **Packer Config**: `raspberry_pi_os_pi5.pkr.hcl`
- **Provision Script**: `resources_raspberry_pi_os/provision_pi_os_pi5.sh`
- **Output Image**: `dexi_raspberry_pi_os_pi5.img`

## Requirements

- Docker with privileged access
- Base image: `bookwork_jazzy_docker_shrinked.img.gz.xz` in `base_images/`
- Minimum 30GB output image size
- ARM64 QEMU static binary for cross-compilation

## Quick Start

1. Build CM4 image:
   ```bash
   ./build_raspberry_pi_os_ark_cm4.sh
   ```

2. Build Pi5 image:
   ```bash
   ./build_raspberry_pi_os_pi5.sh
   ```

## Pre-installed Software

### Core System
- **Base OS**: Raspberry Pi OS Bookworm with ROS2 Jazzy
- **Essential packages**: vim, libi2c-dev
- **I2C support**: Enabled with kernel module auto-loading

### ROS2 Ecosystem
- **ROS2 Jazzy**: Complete installation with workspace setup
- **ROSBridge**: Web interface for ROS communication
- **Micro-ROS Agent**: Microcontroller communication
- **Computer Vision**: AprilTag detection, camera support, YOLO object detection
- **Custom DEXI packages**: Interfaces, LED control, camera, C++ core

### Hardware Support
- **LED Control**:
  - CM4: Adafruit NeoPixel libraries
  - Pi5: pi5neo library
- **Camera**:
  - CM4: CSI camera with libcamera
  - Pi5: USB camera support
- **MAVLink Router**: Drone communication protocol routing

### Networking & Connectivity
- **WiFi Hotspot**: Auto-configured with MAC-based SSID (`dexi_XXXX`)
- **Default Password**: `droneblocks`
- **Network Management**: `dexi-wifi` command for WiFi configuration
- **Hotspot Creation**: Automatic on first boot

### Development Tools
- **Node-RED**: Visual programming interface (DroneBlocks/node-red-dexi)
- **Docker**: Container support with pre-configured services
- **Code-server**: VS Code in browser (if setup script available)

### Services & Configuration
- **DEXI Service**: Auto-start ROS2 nodes on boot
- **MAVLink Router**: Automatic startup with failover
- **Hotspot Setup**: One-time service for WiFi configuration
- **Hardware Configs**: Platform-specific boot configurations

## Platform Differences

| Feature | CM4 (ARK) | Pi5 |
|---------|-----------|-----|
| LED Library | Adafruit NeoPixel | pi5neo |
| Camera | CSI (libcamera) | USB |
| Config Path | `config/cm4/` | `config/pi5/` |
| MAVLink Config | `ark_cm4_main.conf` | `main.conf` |

## Build Process

Images are built using Packer with ARM builder in Docker:
1. Extract base image (30GB partition)
2. Mount boot (256MB FAT) and root (ext4) partitions
3. Copy resources to `/tmp/resources`
4. Execute platform-specific provision script
5. Generate final compressed image

## Network Configuration

- **Hotspot SSID**: `dexi_` + last 4 MAC address characters
- **Default Password**: `droneblocks`
- **Web Dashboard**: Available at http://192.168.4.1 after connecting to hotspot
- **WiFi Setup**: `sudo dexi-wifi 'NetworkName' 'password'`
- **Web Access**: Code-server and Node-RED available via hotspot

## File Structure

```
dexi-os/
├── base_images/                    # Base OS images
├── resources_raspberry_pi_os/      # Provision scripts and configs
├── raspberry_pi_os_*.pkr.hcl      # Packer configurations
├── build_raspberry_pi_os_*.sh     # Build scripts
└── *.img                          # Output images
```
