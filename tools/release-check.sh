#!/usr/bin/env bash
# Check a release is actually finished. Run it before publishing, and again after.
#
# Every check here exists because it was missed on a real release, not because it
# seemed like a good idea. v0.22 shipped with four of these failing.
#
#   ./tools/release-check.sh v0.22
#
# Exit 0 only when everything passes. Anything else means do not publish.

set -uo pipefail

VER="${1:-}"
if [ -z "$VER" ]; then
    echo "usage: $0 vX.Y" >&2
    exit 2
fi

R2_PUBLIC="https://pub-7efc16585b2a4b5ab550489e8d8d5b33.r2.dev"
BUCKET="r2:dexi-os-releases"
ASSETS="$BUCKET/build-assets"
TARGETS="ark_cm4 ark_cm5 cm5 pi5"
REPOS="dexi-os dexi_bringup"

fail=0
ok()   { printf '  \033[32mok\033[0m    %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; fail=$((fail+1)); }
warn() { printf '  \033[33mwarn\033[0m  %s\n' "$1"; }

echo "== $VER =="

# ---------------------------------------------------------------- images
echo
echo "images published under $VER/"
newest_image_epoch=0
for t in $TARGETS; do
    url="$R2_PUBLIC/$VER/dexi_raspberry_pi_os_$t.img.zip"
    hdr=$(curl -sI --max-time 30 "$url")
    code=$(printf '%s' "$hdr" | awk 'NR==1{print $2}')
    if [ "$code" != "200" ]; then
        bad "$t: HTTP $code"
        continue
    fi
    lm=$(printf '%s' "$hdr" | awk 'tolower($1)=="last-modified:"{sub($1" ","");print}' | tr -d '\r')
    epoch=$(date -j -f "%a, %d %b %Y %T %Z" "$lm" +%s 2>/dev/null || echo 0)
    [ "$epoch" -gt "$newest_image_epoch" ] && newest_image_epoch=$epoch
    ok "$t: $lm"
done

# --------------------------------------------------------- container drift
# The image loads containers from THIS tar, not from Docker Hub. An image built
# before the tar was refreshed silently ships the previous container. v0.22
# shipped the 0.21 GCS this way and reported 91% CPU against a true 31%.
echo
echo "container tar older than every image"
tar_line=$(rclone lsl "$ASSETS/dexi-droneblocks.tar" 2>/dev/null | head -1)
if [ -z "$tar_line" ]; then
    bad "dexi-droneblocks.tar not found in build-assets"
else
    tar_date=$(printf '%s' "$tar_line" | awk '{print $2" "$3}' | cut -d. -f1)
    tar_epoch=$(date -j -f "%Y-%m-%d %T" "$tar_date" +%s 2>/dev/null || echo 0)
    if [ "$tar_epoch" -gt "$newest_image_epoch" ]; then
        bad "tar ($tar_date) is NEWER than the newest image — rebuild all four"
    else
        ok "tar $tar_date predates every image"
    fi
fi

# ------------------------------------------------------------ asset freeze
# Without this the release stops being rebuildable the moment the next cycle
# overwrites build-assets. Skipped on v0.22 and only caught after publishing.
echo
echo "build assets frozen under $VER/"
for f in dexi-droneblocks.tar dexi-droneblocks.BUILD_INFO \
         dexi-node-red.tar dexi-node-red.BUILD_INFO \
         bookwork_jazzy_docker_shrinked.img.gz.xz; do
    if rclone lsl "$ASSETS/$VER/$f" >/dev/null 2>&1 && \
       [ -n "$(rclone lsl "$ASSETS/$VER/$f" 2>/dev/null)" ]; then
        ok "$f"
    else
        bad "$f not frozen under build-assets/$VER/"
    fi
done

# ------------------------------------------------------------- merged back
# The runbook has said to do this since v0.21 and it was still missed. main
# ended up without the timezone and fake-hwclock changes entirely.
echo
echo "rc/$VER merged into main"
for r in $REPOS; do
    cmp=$(gh api "repos/DroneBlocks/$r/compare/main...rc/$VER" \
          --jq '"\(.ahead_by) \(.behind_by)"' 2>/dev/null)
    if [ -z "$cmp" ]; then
        warn "$r: no rc/$VER branch"
        continue
    fi
    ahead=$(printf '%s' "$cmp" | cut -d' ' -f1)
    if [ "$ahead" = "0" ]; then
        ok "$r: main has everything from rc/$VER"
    else
        bad "$r: rc/$VER is $ahead commits ahead of main — merge it back"
    fi
done

# -------------------------------------------------------------------- tag
echo
echo "tag and release"
if gh api "repos/DroneBlocks/dexi-os/git/refs/tags/$VER" >/dev/null 2>&1; then
    ok "tag $VER exists"
else
    bad "tag $VER does not exist"
fi
state=$(gh release view "$VER" -R DroneBlocks/dexi-os --json isDraft --jq .isDraft 2>/dev/null)
case "$state" in
    false) ok "release published" ;;
    true)  warn "release is still a draft (fine before publishing)" ;;
    *)     bad "no GitHub release for $VER" ;;
esac

# ----------------------------------------------------------- pinned deps
# An unpinned dexi.repos means two builds of the same version can bake different
# code. dexi_yolo tracked main until v0.22 was already being built.
echo
echo "dexi.repos pinned on rc/$VER"
repos_file=$(gh api "repos/DroneBlocks/dexi_bringup/contents/dexi.repos?ref=rc/$VER" \
             --jq .content 2>/dev/null | base64 -d 2>/dev/null)
if [ -z "$repos_file" ]; then
    warn "could not read dexi.repos from rc/$VER"
else
    floating=$(printf '%s' "$repos_file" | grep -c "version: main" || true)
    if [ "$floating" -gt 0 ]; then
        warn "$floating repo(s) still track main — two builds can differ"
        printf '%s' "$repos_file" | grep -B3 "version: main" | grep -E "^  [a-z_]+:" | sed 's/^/          /'
    else
        ok "no repo tracks a moving branch"
    fi
fi

echo
if [ "$fail" -eq 0 ]; then
    echo "PASS"
else
    echo "$fail check(s) FAILED — do not publish"
fi
exit "$fail"
