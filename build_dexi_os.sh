# https://github.com/mkaczanowski/packer-builder-arm
sudo docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build ./ubuntu_server_22.04_arm64.pkr.hcl