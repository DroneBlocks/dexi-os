# Cutting a DEXI-OS release

Written after v0.21. Every step here exists because skipping it cost time.

## Vocabulary

- `vX.Y-rcN` is a release candidate. Mutable: overwrite the R2 prefix freely.
- `vX.Y` is the release. Once published the URLs are a contract, so never rebuild into that prefix afterward.
- The `version` workflow input is both the R2 prefix and the value baked into `/etc/dexi-version`. Passing `v0.21` when you meant `v0.21-rc8` overwrites the published release.

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

## Verifying a download

```bash
md5 -q <local file>
rclone hashsum md5 r2:dexi-os-releases/<prefix>/<image>
```

Sizes are close enough between builds to be useless for telling images apart. Hash them.
