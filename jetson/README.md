# Dexi on Jetson (Docker)

Runs the Dexi ROS2 stack on NVIDIA Jetson boards via Docker. Tested on Jetson Orin Nano Developer Kit with JetPack 5.1.3 (Ubuntu 20.04).

## Why Docker?

ROS2 Jazzy requires Ubuntu 24.04. The Jetson's host OS (20.04 on JetPack 5, 22.04 on JetPack 6) can't run Jazzy natively. The Docker container runs Ubuntu 24.04 internally, so the host OS version doesn't matter.

## Prerequisites

- NVIDIA Jetson with Docker installed
- USB camera (e.g., Arducam) for vision tasks
- ARK Pi6X flight controller connected via USB (for flight)

## Build

```bash
# Copy jetson/ directory to the Jetson
scp -r jetson/ user@<jetson-ip>:~/dexi-docker/

# SSH in and build (takes ~30 minutes on Orin Nano)
ssh user@<jetson-ip>
cd ~/dexi-docker
chmod +x build.sh
./build.sh
```

The build compiles natively on aarch64 — no QEMU emulation needed, so it's much faster than the Pi image builds.

## Run

```bash
# Interactive shell
docker run -it --rm --privileged --network host -v /dev:/dev dexi-ros2-jazzy

# Inside the container:
ros2 pkg list | grep dexi
```

### USB Camera + AprilTag Detection

```bash
docker run -it --rm --privileged --network host -v /dev:/dev dexi-ros2-jazzy bash

# Inside container:
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 &
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/image_raw \
  -r camera_info:=/camera_info
```

### Full Stack (MAVLink + ROS2)

```bash
docker run -it --rm --privileged --network host -v /dev:/dev dexi-ros2-jazzy bash

# Inside container:
mavlink-routerd &
ros2 launch dexi_bringup dexi.launch.py
```

## Included Packages

| Category | Packages |
|----------|----------|
| AprilTag | `apriltag`, `apriltag_ros`, `apriltag_msgs` |
| Flight Control | `px4_msgs`, `dexi_cpp`, `dexi_offboard`, `dexi_interfaces` |
| Camera | `usb_cam` (V4L2), `dexi_camera` (calibration files) |
| Vision | `cv_bridge`, `image_transport`, all transport plugins |
| AI/ML | `dexi_yolo`, `dexi_color_detection` |
| Comms | `rosbridge_server`, `micro_ros_agent` |
| MAVLink | `mavlink-routerd` |
| Bringup | `dexi_bringup` |

## Excluded Packages (Pi-specific)

| Package | Reason |
|---------|--------|
| `dexi_led` | NeoPixel/pi5neo GPIO hardware |
| `dexi_gpio` | lgpio, Pi-specific |
| `camera_ros` | libcamera `MergePolicy` API mismatch in Ubuntu 24.04; replaced by `usb_cam` |

## Upgrading to JetPack 6 / Ubuntu 22.04

If you reflash the Jetson to JetPack 6.x (Ubuntu 22.04), you have two options:

### Option A: Keep using Docker (recommended for Jazzy)
This Dockerfile works unchanged on any host OS. The container always runs Ubuntu 24.04 internally. No changes needed.

### Option B: Native ROS2 Humble
Ubuntu 22.04 supports ROS2 Humble natively. You would:
1. `sudo apt install ros-humble-desktop`
2. Clone and build the dexi workspace, pinning repos to `humble` branches where available
3. Same package exclusions apply (no dexi_led/gpio)

Note: Humble is EOL May 2027. Jazzy (in Docker) is supported until May 2029.

### Option C: Native ROS2 Jazzy from source on 22.04
Possible but not recommended — many Jazzy deps target Noble (24.04). Docker is easier.

## Architecture Notes

The camera pipeline on Jetson differs from the Pi:

```
Pi:       Arducam → dexi_camera (picamera2) → image topics → apriltag_ros
Jetson:   Arducam → usb_cam (V4L2)          → image topics → apriltag_ros
```

The downstream pipeline (apriltag_ros, dexi_offboard, etc.) is identical. Topic names may need remapping — `usb_cam` publishes to `/image_raw` while `camera_ros` uses `/camera/image_raw`.

## Image Size

The built Docker image is ~3.8GB. Ensure the Jetson has sufficient disk space (the Orin Nano Dev Kit has 116GB eMMC).
