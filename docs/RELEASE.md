# Cutting a DEXI-OS release

Written after v0.21. Every step here exists because skipping it cost time.

## Vocabulary

- `vX.Y-rcN` is a release candidate. Mutable: overwrite the R2 prefix freely.
- `vX.Y` is the release. Once published the URLs are a contract, so never rebuild into that prefix afterward.
- The `version` workflow input is both the R2 prefix and the value baked into `/etc/dexi-version`. Passing `v0.21` when you meant `v0.21-rc8` overwrites the published release.

## Landing a fix in a release

A release spans five repos and the milestone only lives on this one, so:

1. **Open the issue here**, on the `vX.Y` milestone, whichever repo the code is
   in. This is the only place the whole release scope is visible.
2. **Open the PR in the repo that owns the code**, with
   `Refs DroneBlocks/dexi-os#NN` in the body so the issue links back.
3. **Base branch:** `rc/vX.Y` in `dexi-os` and `dexi_bringup`. `main` in
   `node-red-dexi`, `dexi-droneblocks` and `dexi-5-motor-check`, which ship on
   their own container and npm versions rather than the OS cycle.
4. **Verify on hardware first** and put the measurement in the PR. A number
   beats a description.
5. **Say how it reaches a drone** when merging is not enough. Anything in a
   container needs a rebuild and an R2 bridge before a board sees it, which is
   the step most easily forgotten. See "Updating the GCS or node-red container".

## Release candidates

1. Land the work on `rc/vX.Y` in dexi-os and, if launch files changed, in dexi_bringup.
2. Dispatch `build-image.yml` for **all four targets** with `version=vX.Y-rcN`. An RC missing a target is not an RC. Splitting one target across two prefixes is what made a board on the bench unidentifiable during v0.21.
3. Builds serialize on one runner at about 20 min each, so a full set is roughly 80 min.
4. Confirm what each build actually baked, from the run log rather than from intent:

   ```
   Building target=<target> version=<version>
   dexi_bringup at <sha>
   Embedded version: <version>
   Platform marker: <target>
   ```

5. Flash and boot. `ark_cm4` and `pi5` are the usual pair; `cm5` and `ark_cm5` are easy to forget.

## Checking a flashed board

```bash
cat /etc/dexi-version /etc/dexi-platform
git -C ~/dexi_ws/src/dexi_bringup log -1 --format='%h %s'
systemctl is-active dexi mavlink-router mavlink2rest
ls -l /dev/serial/by-id/
journalctl -u dexi -b --no-pager | grep -icE 'process has died|exited with code [1-9]'
vcgencmd measure_temp; vcgencmd get_throttled
```

From your workstation:

```bash
curl -s http://<board>/api/version    # {"os":"vX.Y","platform":"<target>"}
```

Boards accept SSH by password only: `sshpass -p dexi ssh -o StrictHostKeyChecking=no dexi@<ip>`.
All boards ship the same host key, so `StrictHostKeyChecking=no` is required.

`ros2 topic hz` will report nothing. A wlan0 station does not loop back its own
multicast, so DDS discovery fails for the CLI. Measure over rosbridge on :9090
instead; that path is plain TCP and works.

## Publishing

1. Build all four with `version=vX.Y`. They land in the `vX.Y/` prefix.
2. Freeze the build assets, or the release stops being rebuildable as soon as the next cycle overwrites them:

   ```bash
   B=r2:dexi-os-releases/build-assets
   for f in dexi-droneblocks.tar dexi-droneblocks.BUILD_INFO \
            dexi-node-red.tar dexi-node-red.BUILD_INFO \
            bookwork_jazzy_docker_shrinked.img.gz.xz \
            ark_pi6x_default_v1.16.1.px4 ark_pi6x_default_v1.16.2.px4; do
     rclone copyto "$B/$f" "$B/vX.Y/$f"
   done
   ```

3. Merge `rc/vX.Y` into `main` in dexi-os and dexi_bringup. Use a merge commit, not a squash: the release history is worth keeping.
4. Tag `vX.Y` on `main` in both repos.
5. Create the GitHub release against that tag, `--latest`, `--draft` first. Link only the `vX.Y/` prefix. Never hand-edit one row to point at a different prefix; v0.21-rc5 did that and nobody could tell which image was on the bench.
6. Verify every download link returns 200 before publishing.
7. Publish.

## Opening the next cycle

Do this immediately after publishing, not when the next feature lands.

1. Bump `VERSION` on `main`.
2. Repoint `bringup_ref` in `raspberry_pi_os.pkr.hcl` and in `build-image.yml`, both the input default and the fallback in the build step. Leaving these is silent: the first build of the new cycle bakes the previous cycle's bringup and says nothing.
3. Cut `rc/vX.Y+1` in dexi-os and dexi_bringup.

## Updating the GCS or node-red container

The image build stages `dexi-droneblocks.tar` from R2, not from Docker Hub, so a
merged PR in dexi-droneblocks changes nothing on its own.

1. Dispatch `release.yml` in dexi-droneblocks with a tag.
2. Bridge it to R2, keeping a backup:

   ```bash
   rclone copyto r2:.../dexi-droneblocks.tar r2:.../dexi-droneblocks.tar.bak-<date>
   docker pull --platform linux/arm64 droneblocks/dexi-droneblocks:<tag>
   docker save droneblocks/dexi-droneblocks:<tag> > dexi-droneblocks.tar
   rclone copyto dexi-droneblocks.tar r2:.../dexi-droneblocks.tar
   ```

   Pull with an explicit `--platform`. The workflow pushes a manifest list, and a
   bare pull picks the host's architecture rather than the board's.

3. Update `dexi-droneblocks.BUILD_INFO` in the same sitting. It drifted a whole PR
   behind the tar for three months because the bridge and the sidecar write are
   separate manual steps, so every image from June to v0.21-rc6 carried software
   the sidecar did not describe.
4. Rebuild the images that need it.

## Checking the release is actually finished

```bash
./tools/release-check.sh vX.Y
```

Run it before publishing and again after. Exit 0 means finished. Every check in
it exists because it was missed on a real release:

| check | what it caught |
|---|---|
| all four images published | - |
| container tar predates every image | v0.22 shipped the v0.21 GCS container. The image loads containers from R2, not Docker Hub, so a merged PR changes nothing until the tar is refreshed AND the images rebuilt. The status page reported 91% CPU against a true 31%. |
| build assets frozen under `vX.Y/` | skipped on v0.22, noticed only after publishing. Until it is done the release stops being rebuildable the moment the next cycle overwrites `build-assets/`. |
| `rc/vX.Y` merged into main | missed on v0.22 in both repos. main's `provision.sh` had no timezone or fake-hwclock lines at all, so those fixes existed only on the release branch. |
| tag exists, release published | - |
| `dexi.repos` has no floating refs | `dexi_yolo` tracked `main` until v0.22 was already building. Twelve repos still do. |

The order matters and it is the order that goes wrong: **refresh the container
tar first, then build the images, then freeze the assets, then merge back.**
Building before the tar refresh is silent - nothing fails, the images just carry
the previous container.

## Verifying a download

```bash
md5 -q <local file>
rclone hashsum md5 r2:dexi-os-releases/<prefix>/<image>
```

Sizes are close enough between builds to be useless for telling images apart. Hash them.
