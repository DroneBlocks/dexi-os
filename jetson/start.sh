#!/bin/bash
# Start the full Dexi stack on Jetson Orin Nano
#
# Usage:
#   ./start.sh              # Start all services
#   ./start.sh --no-yolo    # Start without YOLO
#   ./start.sh --no-apriltag # Start without AprilTag

set -e

IMAGE_NAME="dexi-ros2-jazzy"
ROS2_CONTAINER="dexi-ros2"
WEB_CONTAINER="dexi-web"

# Parse args
ENABLE_YOLO=true
ENABLE_APRILTAG=true
for arg in "$@"; do
    case $arg in
        --no-yolo) ENABLE_YOLO=false ;;
        --no-apriltag) ENABLE_APRILTAG=false ;;
    esac
done

# Stop any existing containers
echo "Stopping existing containers..."
docker rm -f ${ROS2_CONTAINER} ${WEB_CONTAINER} 2>/dev/null || true
sleep 2

# Build the ROS2 launch command
LAUNCH_CMD="
# Rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 5

# Camera (Arducam 12MP UVC with anti-flicker settings)
ros2 run usb_cam usb_cam_node_exe --ros-args \\
  -p video_device:=/dev/video0 \\
  -p framerate:=15.0 \\
  -p image_width:=1280 \\
  -p image_height:=720 \\
  -p pixel_format:=mjpeg2rgb \\
  -p brightness:=0 \\
  -p autoexposure:=1 \\
  -p exposure:=200 \\
  -p autofocus:=false \\
  -p focus:=144 \\
  -r /image_raw:=/cam0/image_raw \\
  -r /image_raw/compressed:=/cam0/image_raw/compressed \\
  -r /camera_info:=/cam0/camera_info &
sleep 3
"

if [ "$ENABLE_APRILTAG" = true ]; then
    LAUNCH_CMD+="
# AprilTag detection
ros2 run apriltag_ros apriltag_node --ros-args \\
  -r image_rect:=/cam0/image_raw \\
  -r camera_info:=/cam0/camera_info \\
  -r /detections:=/apriltag_detections &
sleep 2
"
fi

if [ "$ENABLE_YOLO" = true ]; then
    LAUNCH_CMD+="
# YOLO detection (CPU, custom model: car/motorcycle/truck/bird/cat/dog)
ros2 launch dexi_yolo yolo_onnx_launch.py num_threads:=4 detection_frequency:=2.0 &
"
fi

LAUNCH_CMD+="
wait
"

# Start ROS2 container
echo "Starting ROS2 container..."
docker run -d --rm --privileged --network host \
    -v /dev:/dev \
    --name ${ROS2_CONTAINER} \
    ${IMAGE_NAME} bash -c "${LAUNCH_CMD}"

# Start web container
echo "Starting web container..."
docker run -d --rm --network host \
    --name ${WEB_CONTAINER} \
    droneblocks/dexi-droneblocks:latest

echo ""
echo "Waiting for services to start..."
sleep 15

# Verify
echo ""
echo "=== Container Status ==="
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
echo ""
echo "=== ROS2 Topics ==="
docker exec ${ROS2_CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && source /home/dexi/dexi_ws/install/setup.bash && ros2 topic list" 2>/dev/null
echo ""
echo "Web UI: http://$(hostname -I | awk '{print $1}'):3000"
echo "Rosbridge: ws://$(hostname -I | awk '{print $1}'):9090"
