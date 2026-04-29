# Local-build wrapper for the pi5 target.
# All three image targets share raspberry_pi_os.pkr.hcl + provision.sh; only the
# `target` HCL variable differs.
sudo docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build -var "target=pi5" ./raspberry_pi_os.pkr.hcl
