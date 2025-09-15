#!/bin/bash

# DEXI Docker Containers Setup Script
# This script sets up Docker containers for DEXI builds

# Logging function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log "Setting up DEXI Docker containers..."

# Add dexi user to docker group for container access
usermod -aG docker dexi

################################ NODE-RED SETUP ################################
log "Setting up Node-RED repository..."
if [ ! -d "/home/dexi/node-red-dexi" ]; then
    git clone https://github.com/DroneBlocks/node-red-dexi /home/dexi/node-red-dexi
    chown -R dexi:dexi /home/dexi/node-red-dexi
    log "Node-RED repository cloned successfully"
else
    log "Node-RED repository already exists"
fi
#################################################################################

################################ DOCKER IMAGE ARCHIVES ################################
# Copy Docker image archives to final location for first-boot loading
log "Setting up Docker image archives for first boot..."

# DEXI DroneBlocks image
if [ -f "/tmp/resources/dexi-droneblocks.tar" ]; then
    log "Copying DEXI DroneBlocks image archive to /home/dexi/"
    mkdir -p /home/dexi
    cp /tmp/resources/dexi-droneblocks.tar /home/dexi/dexi-droneblocks.tar
    chown dexi:dexi /home/dexi/dexi-droneblocks.tar
    log "DEXI DroneBlocks image archive copied successfully"
else
    log "WARNING: DEXI DroneBlocks image archive not found at /tmp/resources/dexi-droneblocks.tar"
fi

# DroneBlocks Node-RED image
if [ -f "/tmp/resources/dexi-node-red.tar" ]; then
    log "Copying DroneBlocks Node-RED image archive to /home/dexi/"
    mkdir -p /home/dexi
    cp /tmp/resources/dexi-node-red.tar /home/dexi/dexi-node-red.tar
    chown dexi:dexi /home/dexi/dexi-node-red.tar
    log "DroneBlocks Node-RED image archive copied successfully"
else
    log "WARNING: DroneBlocks Node-RED image archive not found at /tmp/resources/dexi-node-red.tar"
fi
######################################################################################

################################ IMAGE LOADING SCRIPT ################################
# Create Docker image loading script
cat > /usr/local/bin/load-dexi-images.sh << 'EOF'
#!/bin/bash
echo "Checking for Docker images..."
echo "Current Docker images:"
docker images

# Load DEXI DroneBlocks image
if [ -f /home/dexi/dexi-droneblocks.tar ]; then
    echo "DEXI DroneBlocks archive file found"
    if docker images droneblocks/dexi-droneblocks:latest | grep -q "droneblocks/dexi-droneblocks"; then
        echo "DEXI DroneBlocks image already exists, skipping load"
    else
        echo "DEXI DroneBlocks image not found, loading from archive..."
        if docker load -i /home/dexi/dexi-droneblocks.tar; then
            echo "DEXI DroneBlocks image loaded successfully, removing archive"
            rm /home/dexi/dexi-droneblocks.tar
        else
            echo "Failed to load DEXI DroneBlocks image"
            exit 1
        fi
    fi
else
    echo "No DEXI DroneBlocks archive file found"
fi

# Load DroneBlocks Node-RED image
if [ -f /home/dexi/dexi-node-red.tar ]; then
    echo "DroneBlocks Node-RED archive file found"
    if docker images droneblocks/dexi-node-red:latest | grep -q "droneblocks/dexi-node-red"; then
        echo "DroneBlocks Node-RED image already exists, skipping load"
    else
        echo "DroneBlocks Node-RED image not found, loading from archive..."
        if docker load -i /home/dexi/dexi-node-red.tar; then
            echo "DroneBlocks Node-RED image loaded successfully, removing archive"
            rm /home/dexi/dexi-node-red.tar
        else
            echo "Failed to load DroneBlocks Node-RED image"
            exit 1
        fi
    fi
else
    echo "No DroneBlocks Node-RED archive file found"
fi

echo "Updated Docker images:"
docker images
EOF

chmod +x /usr/local/bin/load-dexi-images.sh
#####################################################################################

################################ CONTAINER STARTUP SCRIPT ################################
# Create container startup script
cat > /usr/local/bin/start-dexi-containers.sh << 'EOF'
#!/bin/bash
echo "Starting DEXI containers..."

# Load Docker images first
/usr/local/bin/load-dexi-images.sh

# Check if containers are already running
if docker ps --format "table {{.Names}}" | grep -q "dexi-droneblocks"; then
    echo "DEXI DroneBlocks container already running"
else
    echo "Starting DEXI DroneBlocks container..."
    docker run -d --restart unless-stopped -p 80:80 --name dexi-droneblocks droneblocks/dexi-droneblocks:latest
    echo "DEXI DroneBlocks container started"
fi

if docker ps --format "table {{.Names}}" | grep -q "dexi-node-red"; then
    echo "DEXI Node-RED container already running"
else
    echo "Starting DEXI Node-RED container..."
    docker run -d --restart unless-stopped -p 1880:1880 \
        -v /home/dexi/node-red-dexi/flows:/data \
        --name dexi-node-red droneblocks/dexi-node-red:latest
    echo "DEXI Node-RED container started"
fi

echo "DEXI containers startup complete"
echo "DroneBlocks available at: http://$(hostname -I | awk '{print $1}')/"
echo "Node-RED available at: http://$(hostname -I | awk '{print $1}'):1880"
EOF

chmod +x /usr/local/bin/start-dexi-containers.sh

################################ ONESHOT SYSTEMD SERVICE ################################
# Create oneshot systemd service to start containers on first boot
cat > /etc/systemd/system/dexi-containers-start.service << 'EOF'
[Unit]
Description=Start DEXI Containers (First Boot)
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/start-dexi-containers.sh

[Install]
WantedBy=multi-user.target
EOF

# Enable the oneshot service
systemctl enable dexi-containers-start.service

log "DEXI Docker containers oneshot service installed"
log "Containers will start once on first boot and restart automatically via Docker"
log "DroneBlocks container will be available at: http://<pi-ip>/"
log "Node-RED container will be available at: http://<pi-ip>:1880"
#################################################################################