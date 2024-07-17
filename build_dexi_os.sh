# https://github.com/mkaczanowski/packer-builder-arm
sudo docker run \
--rm --privileged \
-v /var/run/docker.sock:/var/run/docker.sock \
-v /dev:/dev \
-v /proc:/proc \
-v /sys:/sys \
-v /sys/fs/cgroup:/sys/fs/cgroup:ro \
-v /var/run/dbus:/var/run/dbus \
-v ${PWD}:/build \
mkaczanowski/packer-builder-arm:latest \
build ./ubuntu_server_22.04_arm64.pkr.hcl