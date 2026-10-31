#!/bin/bash
# Mount a built image read-only and run tools/verify_image.py on its root
# partition. Runs in a privileged container, because the runner (macOS +
# Docker) cannot mount ext4 itself.
#
#   tools/verify-image.sh <image.img> <target> <bringup_sha>
set -euo pipefail

IMG=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
TARGET=$2
SHA=$3
REPO=$(cd "$(dirname "$0")/.." && pwd)

docker run --rm --privileged -v /dev:/dev \
    -v "$IMG:/image.img:ro" -v "$REPO:/repo:ro" \
    python:3.12-slim bash -c '
        set -euo pipefail
        LOOP=$(losetup --find --show --read-only --partscan /image.img)
        trap "umount /mnt 2>/dev/null; losetup -d $LOOP" EXIT
        # partscan can lag the losetup return by a moment
        for i in 1 2 3 4 5; do [ -e "${LOOP}p2" ] && break; sleep 1; done
        mount -o ro "${LOOP}p2" /mnt
        python3 /repo/tools/verify_image.py /mnt "$1" "$2"
    ' _ "$TARGET" "$SHA"
