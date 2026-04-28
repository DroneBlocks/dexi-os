# Raspberry Pi 5 as a Build Runner

This guide walks through setting up a Pi 5 as a self-hosted GitHub Actions runner for the DEXI-OS build workflows.

## When to use this

- You want a dedicated, low-power, always-on build machine
- You already have Pi 5 hardware on hand
- Build cadence is occasional (handful of releases per month) — Pi 5 is ~3-4× slower than an Apple Silicon Mac for compile-heavy steps
- You like the architectural symmetry of building Pi images on a Pi

## Hardware

Required:

- **Raspberry Pi 5 — 8GB model.** The 4GB will OOM during `colcon build` when many parallel C++ compilers run.
- **Active cooler.** Builds sustain ~4 cores at 100% for 30-60 min and the SoC will thermal-throttle without one (~80°C threshold). The official Active Cooler is fine.
- **Real PSU (5V/5A USB-C PD).** Underpowered Pis throttle silently and corrupt builds. The official 27W power supply is the simplest answer.
- **Storage** — see below.

Strongly recommended:

- **NVMe via M.2 HAT.** microSD works but is the bottleneck — expect builds to take ~75-95 min vs. ~55-65 min on NVMe. NVMe also has dramatically better write endurance (microSD wears out under sustained writes from CI).
- The official Raspberry Pi M.2 HAT+ is the safest pairing. WD Blue SN580 500GB or TeamGroup MP44L 500GB are both well-tested with Pi 5.

## OS

Use **Raspberry Pi OS Bookworm (64-bit)** — Pi-foundation supported, pre-baked Docker availability, matches the OS our DEXI images target.

Ubuntu Server 24.04 LTS for Pi also works for the runner role. We don't currently use it because the rest of the DEXI ecosystem (camera variants, libcamera integration) is most mature on Pi OS.

## Critical first step: switch to a 4K-page kernel

Pi OS for Pi 5 ships a **16K-page kernel** (`kernel_2712.img`) by default. The `mkaczanowski/packer-builder-arm` Docker image's `packer` Go binary is compiled assuming 4K pages and **segfaults immediately** on a 16K-page kernel. The container loads, qemu binfmt registers, then `packer` exits with code 139 (SIGSEGV) before doing anything useful.

The fix is to boot the generic 4K-page kernel (`kernel8.img`), which already ships in `/boot/firmware/`.

```bash
# Backup config first
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.bak

# Append the kernel directive
sudo bash -c 'cat >> /boot/firmware/config.txt << EOF

# Force 4K-page kernel for Docker/Go binary compatibility (GHA build runner)
kernel=kernel8.img
EOF'

sudo reboot
```

After reboot, verify:

```bash
getconf PAGESIZE                                # should print 4096
uname -r                                        # should show ...+rpt-rpi-v8 (not -2712)
docker run --rm --privileged \
  mkaczanowski/packer-builder-arm:latest version  # should print "Packer v1.10.0"
```

You may lose some Pi-5-specific peripheral optimizations on `kernel8.img` — not a problem for a CI machine, but don't repeat this on a Pi running production drone workloads. To revert, remove the `kernel=` line and reboot.

## Software install

Docker is preinstalled on recent Pi OS Bookworm images. If your Pi was previously a DEXI drone, `docker` will already be working as the user `dexi`. Otherwise:

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
newgrp docker  # or log out and back in
```

Install rclone:

```bash
curl -fsSL https://rclone.org/install.sh | sudo bash
```

Configure rclone with R2 credentials (the file should be `chmod 600`):

```bash
mkdir -p ~/.config/rclone
cat > ~/.config/rclone/rclone.conf << 'EOF'
[r2]
type = s3
provider = Cloudflare
access_key_id = YOUR_ACCESS_KEY
secret_access_key = YOUR_SECRET
endpoint = https://YOUR_ACCOUNT_ID.r2.cloudflarestorage.com
acl = private
EOF
chmod 600 ~/.config/rclone/rclone.conf

# Verify
rclone lsd r2:
```

Pull the packer-builder-arm image (this is what the workflow uses):

```bash
docker pull mkaczanowski/packer-builder-arm:latest
```

## Install the GitHub Actions runner

1. Go to https://github.com/DroneBlocks/dexi-os/settings/actions/runners/new
2. Choose **Linux** + **ARM64**
3. Copy the `--token AAAA...` value from the Configure step

Then on the Pi:

```bash
mkdir -p ~/actions-runner && cd ~/actions-runner
curl -fsSL -o runner.tar.gz \
  https://github.com/actions/runner/releases/download/v2.334.0/actions-runner-linux-arm64-2.334.0.tar.gz
tar xzf runner.tar.gz && rm runner.tar.gz

./config.sh \
  --url https://github.com/DroneBlocks/dexi-os \
  --token YOUR_TOKEN \
  --name pi5-builder \
  --labels pi5 \
  --unattended --replace
```

The `--labels pi5` part is what lets the workflow target this runner via its `runner_label` input.

Start the runner:

```bash
nohup ./run.sh > /tmp/gha-runner.log 2>&1 &
disown
```

Or install as a systemd service so it survives reboots:

```bash
sudo ./svc.sh install $USER
sudo ./svc.sh start
```

Verify it shows `online` at https://github.com/DroneBlocks/dexi-os/settings/actions/runners.

## Triggering a build

```bash
gh workflow run build-cm5.yml \
  -f version=v0.21 \
  -f runner_label=pi5
```

Or via the GitHub UI: Actions → Build CM5 Image → Run workflow → choose `pi5`.

The output lands at:
```
https://pub-7efc16585b2a4b5ab550489e8d8d5b33.r2.dev/<version>/dexi_raspberry_pi_os_cm5.img.zip
```

## Performance expectations

Measured on Pi 5 8GB with microSD (post-4K-kernel switch):

| Step | Pi 5 microSD | Pi 5 NVMe (estimate) | Reference: M2 MacBook |
|---|---|---|---|
| Stage assets from R2 | 1-2 min | 1-2 min | 1-2 min |
| Build CM5 image (packer) | 40-60 min | 25-35 min | 10-12 min |
| Compress image (zip -1) | 8-12 min | 6-8 min | 3-4 min |
| Upload to R2 | ~24 min | ~24 min | ~24 min |
| **Total** | **~75-95 min** | **~55-65 min** | **~40 min** |

Upload time is bandwidth-bound and roughly equal across runners on the same network.

## If you put this Pi on the same network as a DEXI drone

If the Pi was previously imaged as a DEXI drone (with `dexi.service`, `dexi-droneblocks` Docker container, etc.), disable those services so they don't compete with builds:

```bash
sudo systemctl disable --now dexi code-server hailort mavlink-router lttng-sessiond
docker update --restart=no dexi-droneblocks dexi-node-red
docker stop dexi-droneblocks dexi-node-red
```

The DEXI containers in particular will keep auto-starting at boot otherwise.

## Troubleshooting

**packer exits with code 139 (SIGSEGV)** during the build step
→ The 4K-page kernel switch above wasn't applied or reverted. Verify `getconf PAGESIZE` returns `4096`.

**Build OOMs during a `colcon build` step**
→ Either you're on a 4GB Pi 5 (use the 8GB model) or other services are consuming RAM. Stop `dexi.service` and the DEXI Docker containers before building.

**Build is much slower than the 75-95 min estimate**
→ Check `vcgencmd measure_temp` during a build. If you see `temp=80'C` or higher, the Pi is throttling. Add active cooling.

**`docker run --privileged` fails with permission errors on `/dev/loop*`**
→ The user running the runner needs to be in the `docker` group: `sudo usermod -aG docker $USER && newgrp docker`.

**Disk fills up mid-build**
→ The build needs ~50GB peak. On a 64GB microSD with normal Pi OS install (~10-15GB used), you're at the edge. Use 128GB+ or NVMe.
