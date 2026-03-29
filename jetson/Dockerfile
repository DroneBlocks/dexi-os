# Dexi ROS2 Jazzy for NVIDIA Jetson
#
# Builds the Dexi drone stack in a Docker container.
# Works on any Jetson regardless of host OS (JetPack 5 or 6).
#
# Excluded Pi-specific packages: dexi_led, dexi_gpio, camera_ros
# Camera: uses ros-jazzy-usb-cam instead of camera_ros (libcamera API mismatch)
#
# Build: docker build -t dexi-ros2-jazzy .
# Run:   docker run -it --rm --privileged --network host -v /dev:/dev dexi-ros2-jazzy

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# System packages (mirrors apt_packages.sh from Pi builds)
RUN apt-get update && apt-get install -y \
    vim tmux git wget unzip python3-pip python3-vcstool \
    libi2c-dev libtheora-dev \
    meson ninja-build pkg-config gcc g++ \
    libcamera-dev \
    ros-jazzy-ament-cmake-mypy \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /home/dexi/dexi_ws/src
WORKDIR /home/dexi/dexi_ws

# Clone dexi_bringup and import all repos
RUN git clone -b main https://github.com/droneblocks/dexi_bringup src/dexi_bringup \
    && vcs import --input src/dexi_bringup/dexi.repos src/

# Rosbridge Python dependencies
RUN pip install --break-system-packages cbor2

# Source ROS2 for all subsequent build steps
ENV ROS2_SETUP=/opt/ros/jazzy/setup.bash

# Phase 1: Rosbridge
RUN source $ROS2_SETUP && \
    colcon build --packages-select rosbridge_test_msgs && \
    colcon build --packages-select rosbridge_library && \
    colcon build --packages-select rosapi_msgs && \
    colcon build --packages-select rosapi && \
    colcon build --packages-select rosbridge_msgs && \
    colcon build --packages-select rosbridge_server

# Phase 2: Micro ROS agent
RUN source $ROS2_SETUP && \
    colcon build --packages-select micro_ros_msgs && \
    colcon build --packages-select micro_ros_agent

# Phase 3: DEXI interfaces
RUN source $ROS2_SETUP && \
    colcon build --packages-select dexi_interfaces

# Phase 4: PX4 / Flight control (skip lgpio — not needed on Jetson)
RUN source $ROS2_SETUP && source install/setup.bash && \
    colcon build --packages-select px4_msgs && \
    colcon build --packages-select dexi_cpp && \
    colcon build --packages-select dexi_offboard

# OpenCV needed for vision stack
RUN apt-get update && apt-get install -y \
    libopencv-dev libboost-python-dev libboost-dev \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-camera-calibration-parsers \
    ros-jazzy-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# Phase 5: Vision stack (apriltag + image transport)
RUN source $ROS2_SETUP && source install/setup.bash && \
    colcon build --packages-select image_geometry && \
    colcon build --packages-select cv_bridge && \
    colcon build --packages-select apriltag && \
    colcon build --packages-select apriltag_msgs && \
    colcon build --packages-select topic_tools_interfaces && \
    colcon build --packages-select topic_tools && \
    colcon build --packages-select compressed_image_transport && \
    colcon build --packages-select compressed_depth_image_transport && \
    colcon build --packages-select theora_image_transport && \
    colcon build --packages-select zstd_image_transport && \
    colcon build --packages-select image_transport_plugins

# Phase 6: Camera
# camera_ros skipped — libcamera MergePolicy API mismatch on Ubuntu 24.04
# Using ros-jazzy-usb-cam instead for USB cameras on Jetson
# Build dexi_camera for the calibration file (picamera2 runtime dep won't be used)
RUN source $ROS2_SETUP && source install/setup.bash && \
    colcon build --packages-select dexi_camera

# Phase 7: AprilTag detection
# Remove sources for packages we can't build on Jetson — colcon tries to resolve
# them even when they're only exec_depends. We use usb-cam instead of camera_ros.
RUN rm -rf src/camera_ros src/dexi_led src/dexi_gpio
RUN source $ROS2_SETUP && source install/setup.bash && \
    colcon build --packages-select apriltag_ros

# Phase 8: YOLO + color detection
RUN pip install --break-system-packages onnxruntime
RUN source $ROS2_SETUP && source install/setup.bash && \
    colcon build --packages-select dexi_yolo && \
    colcon build --packages-select dexi_color_detection

# Phase 9: Bringup
RUN source $ROS2_SETUP && source install/setup.bash && \
    colcon build --packages-select dexi_bringup

# Mavlink-router
RUN cd /tmp && \
    git clone https://github.com/mavlink-router/mavlink-router && \
    cd mavlink-router && \
    git submodule update --init --recursive && \
    meson setup build . && \
    ninja -C build && \
    ninja -C build install && \
    cd / && rm -rf /tmp/mavlink-router

# Copy mavlink-router config
RUN mkdir -p /etc/mavlink-router && \
    cp /home/dexi/dexi_ws/src/dexi_bringup/config/mavlink-router/ark_cm4_main.conf /etc/mavlink-router/main.conf

# Setup environment in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/dexi/dexi_ws/install/setup.bash" >> /root/.bashrc

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
