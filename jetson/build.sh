#!/bin/bash
# Build the Dexi ROS2 Jazzy Docker image for NVIDIA Jetson
#
# Builds natively on aarch64 — no QEMU emulation needed.
# Takes ~30 minutes on Jetson Orin Nano.
#
# Usage:
#   ./build.sh              # build with default tag
#   ./build.sh v1.0         # build with custom tag

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="dexi-ros2-jazzy"
TAG="${1:-latest}"

echo "Building ${IMAGE_NAME}:${TAG}..."
echo "This will take a while (ROS2 workspace compilation)."
echo ""

docker build -t "${IMAGE_NAME}:${TAG}" "${SCRIPT_DIR}"

echo ""
echo "Build complete: ${IMAGE_NAME}:${TAG} ($(docker image inspect ${IMAGE_NAME}:${TAG} --format='{{.Size}}' | numfmt --to=iec 2>/dev/null || echo 'check with docker images'))"
echo ""
echo "Run with:"
echo "  docker run -it --rm --privileged --network host -v /dev:/dev ${IMAGE_NAME}:${TAG}"
