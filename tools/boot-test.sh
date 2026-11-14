#!/bin/bash
# Boot a built image with systemd-nspawn and check that its services come up.
#
#   sudo tools/boot-test.sh <image.img> <target>
#
# verify_image.py reads the image's files; this boots it. systemd starts every
# unit as on a board, first-boot scripts included, so a service that fails at
# boot fails here instead of on a customer's drone.
#
# Needs a Linux arm64 host with systemd (a Hetzner CAX builder): the image's
# arm64 binaries run natively, no emulation. The image is never written:
# its partitions are mounted read-only under an overlay that is thrown away,
# so a build can boot-test an image and still publish it untouched.
#
# The container has no camera, LED, GPIO, serial port, Wi-Fi or flight
# controller. Units that need them are listed in EXPECTED_FAILED below and
# reported, not failed on. Everything else must start.
set -euo pipefail

IMG=$(realpath "$1")
TARGET=$2
BOOT_TIMEOUT=${BOOT_TIMEOUT:-600}   # first boot loads two Docker images
MACHINE=dexi-boot-test
WORK=$(mktemp -d /var/tmp/boot-test.XXXXXX)
REPORT=${REPORT:-boot-test-$TARGET}

# Units that cannot start without board hardware. Each line: unit, then why.
EXPECTED_FAILED="
dexi-hotspot-setup.service   no wlan0 (also DroneBlocks/dexi-os#106)
dexi-set-hostname.service    reads the wlan0 MAC
mavlink-router.service       no flight controller on USB serial
pigpiod.service              no GPIO
rpi-eeprom-update.service    no EEPROM
systemd-remount-fs.service   root is an overlay, not the SD card
systemd-modules-load.service no kernel modules in a container
"

expected() { echo "$EXPECTED_FAILED" | awk '{print $1}' | grep -qxF "$1"; }

LOOP=""
cleanup() {
    machinectl terminate "$MACHINE" 2>/dev/null || true
    for i in $(seq 1 20); do machinectl status "$MACHINE" >/dev/null 2>&1 || break; sleep 1; done
    umount -R "$WORK/root" 2>/dev/null || true
    umount -R "$WORK/lower" 2>/dev/null || true
    [ -n "$LOOP" ] && losetup -d "$LOOP" 2>/dev/null || true
    rm -rf "$WORK"
}
trap cleanup EXIT

command -v systemd-nspawn >/dev/null || { apt-get update -qq; apt-get install -y -qq systemd-container; }

# Read-only image, writable throwaway overlay on top.
LOOP=$(losetup --find --show --read-only --partscan "$IMG")
for i in 1 2 3 4 5; do [ -e "${LOOP}p2" ] && break; sleep 1; done
mkdir -p "$WORK"/{lower,upper,ovl,root,docker}
mount -o ro "${LOOP}p2" "$WORK/lower"
mount -o ro "${LOOP}p1" "$WORK/lower/boot/firmware" 2>/dev/null \
    || mount -o ro "${LOOP}p1" "$WORK/lower/boot"
mount -t overlay overlay -o "lowerdir=$WORK/lower,upperdir=$WORK/upper,workdir=$WORK/ovl" "$WORK/root"
# Docker's overlay2 cannot sit on an overlay, so /var/lib/docker gets real
# disk, seeded with whatever the image already has there.
[ -d "$WORK/lower/var/lib/docker" ] && cp -a "$WORK/lower/var/lib/docker/." "$WORK/docker/"

echo "=== booting $(basename "$IMG") ($TARGET)"
# --network-veth: a private network. Sharing the host's would let the image's
# NetworkManager reconfigure the builder's own interface.
systemd-nspawn --quiet --boot --machine="$MACHINE" --directory="$WORK/root" \
    --network-veth --capability=all --system-call-filter='@keyring bpf' \
    --property=DeviceAllow='/dev/loop-control rwm' \
    --bind="$WORK/docker:/var/lib/docker" \
    > "$WORK/console.log" 2>&1 &

run() { systemd-run --machine="$MACHINE" --quiet --wait --pipe --collect /bin/bash -c "$1"; }

for i in $(seq 1 60); do run true 2>/dev/null && break; sleep 2; done
run true || { echo "FAIL container never came up"; cat "$WORK/console.log"; exit 1; }

# First boot runs first_boot.sh from /etc/local/runonce.d (rc.local), which
# ends in a reboot, as on a board; nspawn restarts the container in place. The
# boot to test is the one after: the runonce queue is empty and the boot ID
# has held still for a minute.
echo "waiting for first boot and its reboot (up to ${BOOT_TIMEOUT}s)"
deadline=$((SECONDS + BOOT_TIMEOUT)); last=""; since=$SECONDS
while [ $SECONDS -lt $deadline ]; do
    id=$(run "find /etc/local/runonce.d -maxdepth 1 -type f | grep -q . && echo pending || cat /proc/sys/kernel/random/boot_id" 2>/dev/null || true)
    if [ -z "$id" ] || [ "$id" = pending ] || [ "$id" != "$last" ]; then last=$id; since=$SECONDS
    elif [ $((SECONDS - since)) -ge 60 ]; then break; fi
    sleep 5
done
[ $SECONDS -lt $deadline ] || { echo "FAIL boot never settled in ${BOOT_TIMEOUT}s"; tail -50 "$WORK/console.log"; exit 1; }
echo "booted $(run 'journalctl --list-boots --no-pager | tail -n +2 | wc -l') time(s)"
STATE=$(run "timeout 180 systemctl is-system-running --wait" || true)
echo "system state: ${STATE:-still starting after 180s}"

failures=0
check() {
    if [ "$1" = ok ]; then echo "PASS $2"; else echo "FAIL $2"; failures=$((failures + 1)); fi
}

echo "=== failed units"
for unit in $(run "systemctl list-units --failed --plain --no-legend" | awk '{print $1}'); do
    if expected "$unit"; then
        echo "EXPECTED $unit: $(echo "$EXPECTED_FAILED" | grep "^$unit" | cut -c30-)"
    else
        check fail "$unit started"
        run "journalctl -b -u $unit --no-pager -n 30" | sed 's/^/    /'
    fi
done

echo "=== services"
for unit in dexi.service docker.service NetworkManager.service ssh.service; do
    s=$(run "systemctl is-active $unit" || true)
    [ "$s" = active ] && check ok "$unit active" || check fail "$unit active ($s)"
done

echo "=== ROS 2"
# The stack takes a while after dexi.service starts; give it two minutes.
nodes=""
for i in $(seq 1 24); do
    nodes=$(run "export HOME=/home/dexi; source /home/dexi/ros2_jazzy/install/setup.bash; source /home/dexi/dexi_ws/install/setup.bash; timeout 20 ros2 node list 2>/dev/null" || true)
    echo "$nodes" | grep -q rosbridge && break
    sleep 5
done
echo "$nodes" | sed 's/^/    /'
echo "$nodes" | grep -q rosbridge && check ok "rosbridge node running" || check fail "rosbridge node running"
run "timeout 5 bash -c '</dev/tcp/127.0.0.1/9090'" 2>/dev/null \
    && check ok "rosbridge listening on 9090" || check fail "rosbridge listening on 9090"

echo "=== Docker containers"
run "docker ps -a --format '{{.Names}} {{.Status}}'" | sed 's/^/    /' || true
for c in $(run "docker ps -a --filter status=exited --format '{{.Names}}'" 2>/dev/null); do
    echo "    --- $c exited, last log lines:"
    run "docker logs --tail 20 $c 2>&1" | sed 's/^/      /' || true
done
state() { run "docker inspect -f '{{.State.Status}}' $1" 2>/dev/null || echo missing; }
# Up to 2 minutes: first boot loads the images before starting them.
for i in $(seq 1 24); do [ "$(state dexi-node-red)" = running ] && break; sleep 5; done
[ "$(state dexi-node-red)" = running ] && check ok "dexi-node-red running" || check fail "dexi-node-red running ($(state dexi-node-red))"
# dexi-droneblocks bind-mounts /proc/device-tree/model, which only a Pi has,
# so Docker cannot start it here. It must at least exist.
s=$(state dexi-droneblocks)
[ "$s" != missing ] && echo "EXPECTED dexi-droneblocks $s: mounts /proc/device-tree/model" || check fail "dexi-droneblocks container exists"

mkdir -p "$(dirname "$REPORT")"
run "journalctl -b --no-pager" > "$REPORT.journal.log" 2>/dev/null || true
cp "$WORK/console.log" "$REPORT.console.log"

echo "=== $failures failure(s)"
[ "$failures" -eq 0 ]
