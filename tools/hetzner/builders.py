#!/usr/bin/env python3
"""
Hetzner build servers for build-release.yml.

    builders.py create <run_id> <target> [<target> ...]
    builders.py wait <run_id> <target> [<target> ...]
    builders.py delete <run_id>
    builders.py reap <max_hours>

create makes one CAX (ARM) server per target. At first boot it installs Docker,
rclone and the GitHub runner, and registers as a one-job runner labeled
hetzner-<run_id>-<target>. wait blocks until every runner is online, and
fails after 15 minutes, so a server that never boots stops the run instead
of leaving a build job queued for a day. delete removes every server of a run. reap removes
any build server older than max_hours, for runs whose delete never ran.

Environment: HCLOUD_TOKEN, RUNNER_ADMIN_TOKEN (dexi-os Administration: write,
to register runners), GITHUB_REPOSITORY, and R2_ACCESS_KEY_ID,
R2_SECRET_ACCESS_KEY, R2_ENDPOINT for the image upload.
"""
import json
import os
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timedelta, timezone
from pathlib import Path

SERVER_TYPE = "cax31"
LOCATIONS = ["nbg1", "fsn1", "hel1"]  # CAX stock varies; try in order
PURPOSE = "dexi-os-builder"
INIT = Path(__file__).with_name("builder-init.sh")


def api(method, url, token, body=None):
    req = urllib.request.Request(url, method=method, data=json.dumps(body).encode() if body else None,
                                 headers={"Authorization": f"Bearer {token}",
                                          "Content-Type": "application/json",
                                          "Accept": "application/json"})
    try:
        with urllib.request.urlopen(req) as r:
            return json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        return json.loads(e.read() or b"{}") | {"_status": e.code}


def hcloud(method, path, body=None):
    return api(method, "https://api.hetzner.cloud/v1" + path, os.environ["HCLOUD_TOKEN"], body)


def github(method, path, body=None):
    return api(method, "https://api.github.com" + path, os.environ["RUNNER_ADMIN_TOKEN"], body)


def servers(selector):
    return hcloud("GET", f"/servers?label_selector={selector}&per_page=50")["servers"]


def create(run_id, targets):
    repo = os.environ["GITHUB_REPOSITORY"]
    version = github("GET", "/repos/actions/runner/releases/latest")["tag_name"].lstrip("v")
    for target in targets:
        label = f"hetzner-{run_id}-{target}"
        jit = github("POST", f"/repos/{repo}/actions/runners/generate-jitconfig",
                     {"name": label, "runner_group_id": 1, "labels": ["self-hosted", label]})
        if "encoded_jit_config" not in jit:
            sys.exit(f"runner registration for {target} failed: {jit.get('message', jit)}")
        script = INIT.read_text()
        for key, value in {"RUNNER_VERSION": version, "JIT_CONFIG": jit["encoded_jit_config"],
                           "R2_ACCESS_KEY_ID": os.environ["R2_ACCESS_KEY_ID"],
                           "R2_SECRET_ACCESS_KEY": os.environ["R2_SECRET_ACCESS_KEY"],
                           "R2_ENDPOINT": os.environ["R2_ENDPOINT"]}.items():
            script = script.replace(f"__{key}__", value)
        for location in LOCATIONS:
            r = hcloud("POST", "/servers", {
                "name": f"dexi-os-{run_id}-{target.replace('_', '-')}",
                "server_type": SERVER_TYPE, "location": location, "image": "ubuntu-24.04",
                "user_data": script, "labels": {"purpose": PURPOSE, "run_id": str(run_id), "target": target}})
            if "server" in r:
                print(f"{target}: {r['server']['name']} in {location}")
                break
            print(f"{target}: {location} refused: {r.get('error', {}).get('message')}")
        else:
            sys.exit(f"no location could create a server for {target}")


def wait(run_id, targets, minutes=15):
    want = {f"hetzner-{run_id}-{t}" for t in targets}
    deadline = time.time() + minutes * 60
    while time.time() < deadline:
        runners = github("GET", f"/repos/{os.environ['GITHUB_REPOSITORY']}/actions/runners?per_page=100")["runners"]
        online = {r["name"] for r in runners if r["status"] == "online"} & want
        print(f"{len(online)}/{len(want)} runners online", flush=True)
        if online == want:
            return
        time.sleep(20)
    sys.exit(f"runners never came online: {sorted(want - online)}")


def delete(selector, older_than=None):
    for s in servers(selector):
        created = datetime.fromisoformat(s["created"])
        if older_than and datetime.now(timezone.utc) - created < older_than:
            continue
        r = hcloud("DELETE", f"/servers/{s['id']}")
        print(f"deleted {s['name']} (created {s['created']})" if "action" in r else f"FAILED to delete {s['name']}: {r}")


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "create":
        create(sys.argv[2], sys.argv[3:])
    elif cmd == "wait":
        wait(sys.argv[2], sys.argv[3:])
    elif cmd == "delete":
        delete(f"purpose={PURPOSE},run_id={sys.argv[2]}")
    elif cmd == "reap":
        delete(f"purpose={PURPOSE}", timedelta(hours=float(sys.argv[2])))
    else:
        sys.exit(__doc__)
