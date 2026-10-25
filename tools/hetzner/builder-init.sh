#!/bin/bash
# First-boot script for a Hetzner build server (user_data). builders.py fills
# in the __PLACEHOLDERS__. Installs what build-image.yml needs and starts a
# one-job GitHub runner; the server is deleted by build-release.yml afterwards.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update -qq
apt-get install -y -qq docker.io zip unzip rclone

useradd -m -s /bin/bash -G docker runner
echo "runner ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/runner

install -d -o runner -g runner -m 700 /home/runner/.config /home/runner/.config/rclone
cat > /home/runner/.config/rclone/rclone.conf << 'EOF'
[r2]
type = s3
provider = Cloudflare
access_key_id = __R2_ACCESS_KEY_ID__
secret_access_key = __R2_SECRET_ACCESS_KEY__
endpoint = __R2_ENDPOINT__
acl = private
EOF
chown runner:runner /home/runner/.config/rclone/rclone.conf
chmod 600 /home/runner/.config/rclone/rclone.conf

sudo -u runner bash -c 'mkdir -p ~/actions-runner && cd ~/actions-runner &&
    curl -sL https://github.com/actions/runner/releases/download/v__RUNNER_VERSION__/actions-runner-linux-arm64-__RUNNER_VERSION__.tar.gz | tar xz'

# Pulled here so the build does not wait on it.
docker pull -q mkaczanowski/packer-builder-arm:latest

systemd-run --unit=gh-runner --uid=runner --gid=runner \
    -p WorkingDirectory=/home/runner/actions-runner -p Environment=HOME=/home/runner \
    /home/runner/actions-runner/run.sh --jitconfig __JIT_CONFIG__
