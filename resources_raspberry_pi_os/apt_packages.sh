#!/bin/bash

# Logging function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Common apt packages for all DEXI builds
install_common_packages() {
    log "Installing common apt packages..."
    apt-get install -y vim libi2c-dev tmux >/dev/null 2>&1
    apt remove modemmanager -y >/dev/null 2>&1
    apt install -y libtheora-dev >/dev/null 2>&1
    apt install -y meson ninja-build pkg-config gcc g++ systemd >/dev/null 2>&1
    log "Common apt packages installed successfully"
}

# Camera packages (CM4/CM5)
install_camera_packages() {
    log "Installing camera packages..."
    apt install -y libcamera-dev >/dev/null 2>&1
    log "Camera packages installed successfully"
}
