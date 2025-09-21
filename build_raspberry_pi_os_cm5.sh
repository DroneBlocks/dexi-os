# https://github.com/mkaczanowski/packer-builder-arm
sudo docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build ./raspberry_pi_os_cm5.pkr.hcl
