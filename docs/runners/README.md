# Build Runners

The DEXI-OS GitHub Actions workflows (`build-cm5.yml`, `build-ark-cm4.yml`, etc.) build the OS images by running `packer-builder-arm` inside a Docker container, compressing the result, and uploading it to Cloudflare R2 (`dexi-os-releases` bucket).

These builds need ~50GB peak disk, ~8GB RAM, native ARM64 (or amd64 with QEMU), and privileged Docker. GitHub-hosted standard runners don't fit (only ~14GB free disk). We use **self-hosted runners** instead.

## Currently supported runner platforms

| Platform | Guide | Native ARM64 | Notes |
|---|---|---|---|
| Raspberry Pi 5 (8GB) | [pi5.md](pi5.md) | ✅ | Cheapest option (~$170 with NVMe). microSD works but slow |
| macOS (Mac mini or dev Mac, Apple Silicon) | TBD | ✅ | Fastest option. Headless setup uses colima (not Docker Desktop) |
| Linux/amd64 (Intel/AMD PC, WSL2, server) | TBD | ❌ — uses QEMU | Slower (~1.5-2× vs ARM hosts) but always-available hardware |

## Runner labels

Each runner is registered with a custom label so workflows can target a specific machine via the `runner_label` workflow input. Current labels in use:

- `pi5` — the Raspberry Pi 5 build runner
- `macOS` — any Apple Silicon Mac runner (default GitHub label)
- `mac-mini-ny` — the dedicated Mac mini build server (planned)

## Build assets in R2

The build pulls these gitignored assets from `r2:dexi-os-releases/build-assets/` at runtime so the workflow stays portable across runners:

- `bookwork_jazzy_docker_shrinked.img.gz.xz` — pre-built Pi OS Bookworm + ROS2 Jazzy base image (~3.4GB)
- `dexi-droneblocks.tar` — DEXI DroneBlocks Docker image
- `dexi-node-red.tar` — DEXI Node-RED Docker image
- `ark_pi6x_default_v1.16.1.px4` — ARK PX4 firmware

If the R2 bucket is ever rebuilt, repopulate from a known-good runner using `rclone copy`.

## Security note

Self-hosted runners on a public repo are a real risk if any workflow runs on `pull_request` from forks (a fork's PR could execute arbitrary code on your runner with your user's permissions). All current workflows are `workflow_dispatch`-only, which is safe. **Before adding any `pull_request`-triggered workflow to this repo, restrict it to GitHub-hosted runners (`runs-on: ubuntu-latest`).**

The repo is also configured under Settings → Actions → General to require approval for fork PR workflows from all external contributors.
