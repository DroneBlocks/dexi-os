#!/bin/bash

# DEXI Code-Server Setup Script
# This script sets up VS Code Server for DEXI builds

# Logging function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log "Setting up code-server..."

# Download and install code-server
curl -fsSL https://code-server.dev/install.sh | sh

# Create config directory for dexi user
mkdir -p /home/dexi/.config/code-server

# Create the config.yaml file
cat > /home/dexi/.config/code-server/config.yaml << 'EOF'
bind-addr: 0.0.0.0:9999
auth: password
password: droneblocks
cert: false
user-data-dir: /home/dexi/.local/share/code-server
extensions-dir: /home/dexi/.local/share/code-server/extensions
EOF

# Set dark theme as default
mkdir -p /home/dexi/.local/share/code-server/User
cat > /home/dexi/.local/share/code-server/User/settings.json << 'EOF'
{
    "workbench.colorTheme": "Default Dark+",
    "workbench.startupEditor": "none"
}
EOF

# Create systemd service file
cat > /etc/systemd/system/code-server.service << 'EOF'
[Unit]
Description=code-server
After=network.target

[Service]
Type=simple
User=dexi
Environment=HOME=/home/dexi
ExecStart=/usr/bin/code-server --config /home/dexi/.config/code-server/config.yaml /home/dexi
Restart=always

[Install]
WantedBy=multi-user.target
EOF

# Enable the service
systemctl enable code-server

log "Code-server setup completed"
log "VS Code will be available at: http://<pi-ip>:9999"
log "Login password: droneblocks"