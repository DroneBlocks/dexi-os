# Mac mini as a Build Runner

This guide walks through setting up a dedicated Mac mini (Apple Silicon) as a self-hosted GitHub Actions runner for the DEXI-OS build workflows. This is the **fastest** runner option we currently support — about 3× faster end-to-end than a Pi 5.

## When to use this

- You want the fastest possible build turnaround
- You have (or can buy) a dedicated Mac mini that doesn't need to do anything else
- You're OK administering a macOS machine remotely (it doesn't need a monitor or keyboard, but does need `Remote Login` enabled)

## Hardware

Tested configuration:

- **Mac mini, M4, 16 GB RAM, 256 GB SSD or larger.** M2/M3 also fine; 8GB RAM is too tight (we allocate 8GB to the colima VM).
- **Wired Ethernet recommended.** R2 upload speeds especially benefit — datacenter-grade upload bandwidth shaved 15+ min off our build vs. residential Pi.
- macOS 14+ (Sonoma or newer).

The Mac mini does not need a display attached after initial setup. Enabling Remote Login (`System Settings → General → Sharing → Remote Login`) is required so you can SSH in headless.

## Why colima, not Docker Desktop

Docker Desktop on macOS is GUI-driven, requires user-session login to keep the daemon alive, has license/EULA prompts, and is generally a poor fit for a headless build server. **Use colima instead** — it runs Docker inside a Lima-managed Linux VM via macOS Virtualization.framework, fully scriptable, no GUI, no license. Performance is comparable to Docker Desktop and only ~10-20% behind native Linux Docker.

## Prerequisites that must be done at the keyboard (or via Screen Sharing / ARD)

These two steps need someone in front of (or screen-sharing into) the Mac. Subsequent setup can be done over SSH.

1. **Make the runner user an Administrator.** `System Settings → Users & Groups → click ⓘ next to the user → toggle on "Allow this user to administer this computer"`. Homebrew's installer specifically checks for `admin` group membership and refuses otherwise.

2. **Install Xcode Command Line Tools.** Open Terminal, run `xcode-select --install`, click **Install** in the GUI dialog, wait ~10 min. Required for Homebrew's compile fallback.

## Optional but recommended: passwordless sudo for the runner user

The Homebrew installer in non-interactive mode (`NONINTERACTIVE=1`) does `sudo -n -v` (no-prompt sudo check), which fails over SSH because the sudo password cache from `sudo -S` doesn't reliably propagate to subshells without a TTY. The standard convention for build-runner accounts is passwordless sudo, since the runner user often needs `sudo` for installs and setup operations.

```bash
echo "<runner-user> ALL=(ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/<runner-user>-runner
sudo chmod 440 /etc/sudoers.d/<runner-user>-runner
sudo visudo -c
```

Trade-off: anyone who compromises the runner user account has unrestricted root. Mitigate by gating SSH access (Tailscale, Netbird, VPN, etc.) and not exposing this machine on the open internet.

## Software install

After the prerequisites above, everything else can run over SSH.

```bash
# Homebrew
NONINTERACTIVE=1 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
eval "$(/opt/homebrew/bin/brew shellenv)"
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile

# Docker (CLI), colima, rclone
brew install colima docker rclone

# Start colima with build-friendly resources
colima start --cpu 4 --memory 8 --disk 80
```

`colima start` downloads a Lima VM image on first run (~600MB, takes a few minutes). Subsequent starts are nearly instant.

Verify everything works:

```bash
docker version
docker run --rm --privileged alpine sh -c 'ls /dev/loop* | head -2'
docker run --rm --privileged mkaczanowski/packer-builder-arm:latest version
```

The packer version output should print "Packer v1.10.0 [linux arm64]" — confirming native ARM64 execution inside the VM. No QEMU overhead.

## rclone configuration

Drop your R2 credentials into `~/.config/rclone/rclone.conf`:

```ini
[r2]
type = s3
provider = Cloudflare
access_key_id = YOUR_ACCESS_KEY
secret_access_key = YOUR_SECRET
endpoint = https://YOUR_ACCOUNT_ID.r2.cloudflarestorage.com
acl = private
```

```bash
chmod 600 ~/.config/rclone/rclone.conf
rclone lsd r2:   # should list the buckets
```

## Install the GitHub Actions runner

1. Go to `https://github.com/<owner>/<repo>/settings/actions/runners/new`
2. Choose **macOS** + **ARM64**
3. Copy the `--token AAAA...` value

Then on the Mac mini:

```bash
mkdir -p ~/actions-runner && cd ~/actions-runner
curl -fsSL -o runner.tar.gz \
  https://github.com/actions/runner/releases/download/v2.334.0/actions-runner-osx-arm64-2.334.0.tar.gz
tar xzf runner.tar.gz && rm runner.tar.gz

./config.sh \
  --url https://github.com/<owner>/<repo> \
  --token YOUR_TOKEN \
  --name mac-mini-<location> \
  --labels mac-mini-<location> \
  --unattended --replace
```

The custom label is what lets workflows target this runner via the `runner_label` workflow input.

## Auto-start as a launchd service

Unlike the Linux runner, the macOS install includes `svc.sh` once registration completes. Use it:

```bash
./svc.sh install
./svc.sh start
./svc.sh status
```

This installs a per-user **LaunchAgent** at `~/Library/LaunchAgents/actions.runner.<owner>-<repo>.<runner-name>.plist`. The runner starts automatically when the user logs in, and survives reboots **as long as that user is set to auto-login** (System Settings → Users & Groups → Automatic login).

If the Mac mini is going to be powered off and on without an interactive login, configure auto-login. Otherwise the runner won't come back online after a power event.

## Triggering a build

```bash
gh workflow run build-cm5.yml \
  -f version=v0.21 \
  -f runner_label=mac-mini-<location>
```

Or via the GitHub UI: Actions → Build CM5 Image → Run workflow → choose the appropriate label.

## Critical gotcha: `$RUNNER_TEMP` for the packer temp mount

The DEXI-OS build workflow bind-mounts a host directory into the packer container at `/tmp` (the packer-builder-arm image needs space there for its xz decompress step — ~19GB intermediate file).

By default, **colima only auto-mounts `$HOME` and `/tmp/colima` from the macOS host into the Lima VM** — it does NOT auto-mount `/tmp`. Docker Desktop, by contrast, auto-mounts more locations including `/tmp`. So a workflow that mounts `-v /tmp/packer-build:/tmp` works on Docker Desktop and native Linux but **silently fails on colima with "No space left on device"** (because the bind mount points to a non-shared path inside the VM, falling back to a small overlay filesystem).

The DEXI-OS workflows use `$RUNNER_TEMP` instead, which lives under the runner's `_work/_temp` directory — under `$HOME`, so colima auto-mounts it. This works on all three runner platforms (Pi 5, Mac mini via colima, dev Mac via Docker Desktop).

If you're adapting these workflows for other build environments, keep this constraint in mind. Anything you bind-mount on a colima host must be under `$HOME` or `/tmp/colima`.

## Performance

Measured on Mac mini M4, 16GB RAM, SSD, gigabit ethernet, colima with 4 CPU / 8GB memory / 80GB disk:

| Step | Duration |
|---|---|
| Stage assets from R2 | ~3-4 min |
| Build CM5 / ARK CM4 image (packer) | ~13 min |
| Compress image (zip -1) | ~3 min |
| Upload to R2 | ~5 min (datacenter-grade upload) |
| **Total** | **~25 min** |

About 3× faster than a Pi 5 on microSD, and noticeably faster than an M2 MacBook running Docker Desktop on residential network.

## Troubleshooting

**`colima start` hangs or fails**
→ Check `colima status` and `colima delete && colima start --cpu 4 --memory 8 --disk 80` to recreate the VM. The Lima VM image download (~600MB) needs to complete on first run.

**`docker run` returns "Cannot connect to the Docker daemon"**
→ colima isn't running. `colima start` again. If the colima service should auto-start, run `brew services start colima` (note: this is separate from the GHA runner LaunchAgent).

**Build fails with "No space left on device" during xz decompress**
→ Confirm the workflow bind-mounts a path under `$HOME` (e.g., `$RUNNER_TEMP/packer-build`) into `/tmp`, not `/tmp/packer-build` directly. See the colima gotcha section above.

**Runner status shows offline after a reboot**
→ Auto-login isn't enabled, so the user-session LaunchAgent didn't fire. Either enable auto-login, or convert the runner to a system-wide LaunchDaemon (more setup, runs at boot regardless of user login).

**Build is slow even on M4**
→ Check `colima status` for `mountType`. `virtiofs` (the default on macOS 13.5+) is fast. If you somehow ended up on `9p` or `sshfs`, recreate with `--mount-type virtiofs`.
