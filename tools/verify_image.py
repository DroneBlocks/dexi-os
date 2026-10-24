#!/usr/bin/env python3
"""
Check a built DEXI-OS root filesystem before it is published.

    python3 tools/verify_image.py <root> <target> <bringup_sha>

<root> is the image's root partition, mounted. Each check catches a bug that
once shipped an image whose ROS stack never started, while the build passed:

  - /etc/dexi-platform names the target, and start.bash has a case arm for it
  - the launch file that arm starts is installed
  - the target has its dexi_config_<target>.yaml
  - every key in config/dexi_config*.yaml is one start.bash reads
  - every package provision.sh builds is in dexi_ws/install/
  - the baked dexi_bringup is the commit the build asked for

Exits 1 and lists every failure.
"""
import re
import sys
from pathlib import Path

PROVISION = Path(__file__).resolve().parent.parent / "resources_raspberry_pi_os" / "provision.sh"

root, target, want_sha = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
ws = root / "home/dexi/dexi_ws"
bringup = ws / "src/dexi_bringup"
start = (bringup / "scripts/start.bash").read_text()
failures = []


def check(ok, message):
    print(("PASS " if ok else "FAIL ") + message)
    if not ok:
        failures.append(message)


# Platform marker, and the case arm start.bash takes for it.
platform_file = root / "etc/dexi-platform"
platform = platform_file.read_text().strip() if platform_file.exists() else ""
check(platform == target, f"/etc/dexi-platform is '{platform}', expected '{target}'")

case = re.search(r'case "\$PLATFORM" in\n(.*?)\nesac', start, re.S).group(1)
arm = next((body for pats, body in re.findall(r"^\s+([\w|]+)\)\n(.*?);;", case, re.M | re.S)
            if platform in pats.split("|")), None)
check(arm is not None, f"start.bash has a case arm for '{platform}'")
if arm:
    launch = re.search(r"ros2 launch dexi_bringup (\S+)", arm).group(1)
    check((ws / "install/dexi_bringup/share/dexi_bringup/launch" / launch).exists(),
          f"{launch} is installed")

check((bringup / f"config/dexi_config_{platform}.yaml").exists(), f"config/dexi_config_{platform}.yaml exists")

# Config keys: two-space node names under nodes:, four-space parameters below them.
read = set(re.findall(r'get_config_value "(\w+)" "(\w+)"', start))
for cfg in sorted((bringup / "config").glob("dexi_config*.yaml")):
    node, in_nodes = None, False
    for line in cfg.read_text().splitlines():
        if re.match(r"^\S", line):
            in_nodes = line.startswith("nodes:")
        elif in_nodes and (m := re.match(r"^  (\w+):", line)):
            node = m.group(1)
        elif in_nodes and node and (m := re.match(r"^    (\w+):", line)):
            check((node, m.group(1)) in read, f"{cfg.name}: nodes.{node}.{m.group(1)} is read by start.bash")

# Packages. Only builds at column 0 run on every target; indented ones sit
# inside a per-target if or case.
for pkg in re.findall(r"^colcon build --packages-select (\S+)", PROVISION.read_text(), re.M):
    check((ws / "install" / pkg).is_dir(), f"{pkg} is installed")

lock = root / "etc/dexi-manifest.lock"
m = re.search(r"dexi_bringup:\n(?:\s+\w+: .*\n)*?\s+version: (\w+)", lock.read_text()) if lock.exists() else None
baked = m.group(1) if m else "missing"
check(baked == want_sha, f"dexi_bringup is {baked[:7]}, expected {want_sha[:7]}")

print(f"\n{len(failures)} failure(s)")
sys.exit(1 if failures else 0)
