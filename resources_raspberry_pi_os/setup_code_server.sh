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

# Hand the data dirs to dexi BEFORE installing anything as that user. The
# mkdir above runs as root, so `sudo -u dexi ... --install-extension` cannot
# create the extensions/ subdirectory inside a root-owned tree and dies with
# EACCES. This ordering was reversed in v0.21-rc2 and the extension silently
# never shipped.
chown -R dexi:dexi /home/dexi/.config/code-server /home/dexi/.local/share/code-server

# Install Python extension (provides the Run button + language features).
# Pulls in ms-python.debugpy and ms-python.vscode-python-envs as transitive deps.
log "Installing ms-python.python extension..."
sudo -u dexi HOME=/home/dexi code-server --install-extension ms-python.python
chown -R dexi:dexi /home/dexi/.local/share/code-server/extensions

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