# Dexi OS Build System

## Overview

This repo builds Raspberry Pi OS images for Dexi drones using [packer-builder-arm](https://github.com/mkaczanowski/packer-builder-arm) inside Docker. The build takes a pre-baked base image (Bookworm + ROS2 Jazzy + Docker) and provisions it with all Dexi-specific packages, services, and configurations.

## Supported Platforms

| Platform | Build Script | Packer HCL | Provision Script | Output Image |
|----------|-------------|-------------|-----------------|--------------|
| **ARK CM4** | `build_raspberry_pi_os_ark_cm4.sh` | `raspberry_pi_os_ark_cm4.pkr.hcl` | `resources_raspberry_pi_os/provision_ark_cm4.sh` | `dexi_raspberry_pi_os_ark_cm4.img` (30GB, ~6GB zipped) |
| CM5 | `build_raspberry_pi_os_cm5.sh` | `raspberry_pi_os_cm5.pkr.hcl` | `resources_raspberry_pi_os/provision_pi_os_cm5.sh` | `dexi_raspberry_pi_os_cm5.img` |
| Pi 5 | `build_raspberry_pi_os_pi5.sh` | `raspberry_pi_os_pi5.pkr.hcl` | `resources_raspberry_pi_os/provision_pi_os_pi5.sh` | `dexi_raspberry_pi_os_pi5.img` |
| Jetson (Docker) | `jetson/build.sh` | N/A (Dockerfile) | `jetson/Dockerfile` | `dexi-ros2-jazzy` Docker image (~3.8GB) |

**Jetson notes**: Runs ROS2 Jazzy in Docker on any JetPack version. Excludes `dexi_led`, `dexi_gpio`, `camera_ros` (Pi-specific). Uses `usb_cam` for cameras. See `jetson/README.md`.

## Build Prerequisites

- Docker running locally
- Base image at `base_images/bookwork_jazzy_docker_shrinked.img.gz.xz` (3.4GB)
- Docker container tars in `resources_raspberry_pi_os/`:
  - `dexi-droneblocks.tar` (189MB) — DroneBlocks web UI, port 80
  - `dexi-node-red.tar` (335MB) — Node-RED, port 1880
- PX4 firmware: `resources_raspberry_pi_os/ark_pi6x_default_v1.16.1.px4`

## Running a Build (ARK CM4)

```bash
./build_raspberry_pi_os_ark_cm4.sh
```

This runs: `docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build ./raspberry_pi_os_ark_cm4.pkr.hcl`

Build time: several hours (full ROS2 workspace compilation via QEMU ARM emulation).

## Build Pipeline Detail

### 1. Packer Setup
- Decompresses base image from XZ (3.4GB → ~20GB → 30GB resized)
- Creates DOS/MBR image: 256MB FAT32 boot + ext4 root
- Enters QEMU aarch64 chroot

### 2. File Provisioning
- Copies `resources_raspberry_pi_os/` → `/tmp/resources/` inside the image

### 3. Provision Script (`provision_ark_cm4.sh`)
Runs inside the chroot and does everything below in order:

#### a. System Setup
- DNS resolver (1.1.1.1)
- APT updates + common packages (via `apt_packages.sh`: vim, libi2c-dev, tmux, meson, ninja-build, etc.)
- i2c-dev kernel module

#### b. ROS2 Workspace — the core of the build
```bash
git clone -b main https://github.com/droneblocks/dexi_bringup /home/dexi/dexi_ws/src/dexi_bringup
vcs import --input /home/dexi/dexi_ws/src/dexi_bringup/dexi.repos /home/dexi/dexi_ws/src/
```
Then builds ~30 ROS2 packages via `colcon build --packages-select` in dependency order.

#### c. Additional Software
- **mavlink-router** — built from source, config from `dexi_bringup/config/mavlink-router/ark_cm4_main.conf`
- **ARK companion scripts** — cloned from `DroneBlocks/ark_companion_scripts`
- **dexi-networking** — hotspot setup (SSID: `dexi_<MAC>`, password: `droneblocks`)
- **Docker containers** — copies tars, creates first-boot loading service
- **code-server** — VS Code on port 9999 (password: `droneblocks`)
- **dexi-mavsdk** — cloned for MAVSDK examples

## Git Repos & Branches

### Cloned directly in provision script
| Repo | Branch | Pinned? |
|------|--------|---------|
| `droneblocks/dexi_bringup` | `main` | Yes (explicit) |
| `mavlink-router/mavlink-router` | default | No — uses latest |
| `DroneBlocks/ark_companion_scripts` | default | No — uses latest |
| `DroneBlocks/dexi-networking` | default | No — uses latest |
| `DroneBlocks/dexi-mavsdk` | default | No — uses latest |
| `DroneBlocks/node-red-dexi` | default | No — uses latest |

### Pulled via `dexi.repos` (vcs import)
| Repo | Branch | Category |
|------|--------|----------|
| `DroneBlocks/dexi_tools` | main | Dexi |
| `DroneBlocks/dexi_cpp` | main | Dexi |
| `DroneBlocks/dexi_led` | main | Dexi |
| `DroneBlocks/dexi_interfaces` | main | Dexi |
| `DroneBlocks/dexi_offboard` | main | Dexi |
| `DroneBlocks/dexi_apriltag` | main | Dexi |
| `DroneBlocks/dexi_gpio` | main | Dexi |
| `DroneBlocks/dexi_camera` | main | Dexi |
| `DroneBlocks/dexi_yolo` | main | Dexi |
| `DroneBlocks/dexi_ctf` | main | Dexi |
| `AprilRobotics/apriltag` | master | Third-party |
| `christianrauch/apriltag_ros` | **c81c05f** (pinned commit) | Third-party |
| `christianrauch/apriltag_msgs` | master | Third-party |
| `fkie/async_web_server_cpp` | ros2-develop | Third-party |
| `ros-perception/vision_opencv` | rolling | Third-party |
| `RobotWebTools/web_video_server` | ros2 | Third-party |
| `christianrauch/camera_ros` | main | Third-party |
| `RobotWebTools/rosbridge_suite` | ros2 | Third-party |
| `ros-perception/image_transport_plugins` | jazzy | Third-party |
| `px4/px4_msgs` | release/1.16 | PX4 |
| `micro-ROS/micro-ROS-Agent` | jazzy | micro-ROS |
| `micro-ROS/micro_ros_msgs` | jazzy | micro-ROS |
| `ros-tooling/topic_tools` | jazzy | ROS2 tools |

### Packages in dexi.repos but NOT built in provision_ark_cm4.sh
- `dexi_tools` — not built
- `dexi_apriltag` — not built (but `apriltag_ros` is)
- `dexi_ctf` — not built
- `async_web_server_cpp` — not built
- `web_video_server` — not built

## Colcon Build Order (ARK CM4)

The provision script builds packages individually in this order:
1. rosbridge_test_msgs → rosbridge_library → rosapi_msgs → rosapi → rosbridge_msgs → rosbridge_server
2. micro_ros_msgs → micro_ros_agent
3. dexi_interfaces
4. dexi_led (with Adafruit NeoPixel for CM4)
5. dexi_gpio
6. px4_msgs → dexi_cpp → dexi_offboard (requires lgpio built from source)
7. image_geometry → cv_bridge → apriltag → apriltag_msgs → topic_tools_interfaces → topic_tools → compressed_image_transport → compressed_depth_image_transport → theora_image_transport → zstd_image_transport → image_transport_plugins
8. camera_ros → dexi_camera
9. apriltag_ros
10. dexi_yolo (requires onnxruntime pip package)
11. dexi_bringup

## Services Installed on Image

| Service | Type | Purpose |
|---------|------|---------|
| `dexi.service` | persistent | Main ROS2 bringup (runs as root, auto-detects CM4/CM5/Pi5) |
| `mavlink-router.service` | persistent | MAVLink routing from FC USB to UDP endpoints |
| `code-server.service` | persistent | VS Code web IDE on port 9999 |
| `dexi-containers-start.service` | oneshot (first boot) | Loads Docker tars and starts containers |
| `dexi-hotspot-setup.service` | oneshot (first boot) | Creates WiFi hotspot with MAC-based SSID |

## Platform Differences (CM4 vs CM5)

| Aspect | CM4 | CM5 |
|--------|-----|-----|
| dexi_bringup branch | main (explicit) | main (default) |
| Boot config | `config/cm4/` | `config/cm5/` |
| LED library | Adafruit NeoPixel | pi5neo |
| dexi_offboard | Built | NOT built |
| Mavlink router config | `ark_cm4_main.conf` | `main.conf` |
| Docker containers | droneblocks + node-red | droneblocks + node-red + px4-gazebo-headless |

## CI/CD

GitHub Actions workflow at `.github/workflows/build-ark-cm4.yml`:
- **Trigger**: Manual (workflow_dispatch)
- **Runner**: self-hosted (currently local Mac)
- **Steps**: checkout → docker build → zip → upload artifact (7-day retention)
- **Note**: Has hardcoded local path for base_images mount

## Known Issues / Cautions

- `set -e` is commented out in `provision_ark_cm4.sh` — build errors are silently ignored
- `exec 2>/dev/null` suppresses all stderr — errors are invisible during build
- Several repos are cloned without branch pins (mavlink-router, ark_companion_scripts, etc.) — builds may not be reproducible
- The base image (`bookwork_jazzy_docker_shrinked.img.gz.xz`) is not version-controlled or hosted — lives only in local `base_images/`
- Output images are 26-30GB on disk, ~6GB compressed — too large for git

## File Structure

```
dexi-os/
├── build_raspberry_pi_os_ark_cm4.sh      # Build entry point
├── raspberry_pi_os_ark_cm4.pkr.hcl       # Packer config
├── base_images/
│   └── bookwork_jazzy_docker_shrinked.img.gz.xz  # Base image (3.4GB)
├── resources_raspberry_pi_os/
│   ├── apt_packages.sh                   # Shared apt package functions
│   ├── provision_ark_cm4.sh              # CM4 provisioning (main build logic)
│   ├── provision_pi_os_cm5.sh            # CM5 provisioning
│   ├── provision_pi_os_pi5.sh            # Pi5 provisioning
│   ├── setup_code_server.sh              # VS Code server setup
│   ├── setup_docker_containers.sh        # Docker container setup (CM4/Pi5)
│   ├── setup_docker_containers_cm5.sh    # Docker container setup (CM5)
│   ├── dexi-droneblocks.tar              # DroneBlocks web UI container
│   ├── dexi-node-red.tar                 # Node-RED container
│   ├── px4-gazebo-headless.tar           # PX4 SITL container (CM5 only)
│   └── ark_pi6x_default_v1.16.1.px4     # PX4 firmware
├── .github/workflows/
│   └── build-ark-cm4.yml                 # CI workflow (self-hosted runner)
└── dexi_raspberry_pi_os_ark_cm4.img      # Build output (not committed)
```
