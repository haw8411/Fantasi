#!/usr/bin/env python3
"""Verify launching an app leaks nothing: the heap returns to the same value
after every run.

The launcher must reclaim the app image, its task stack/TCB, and everything the
app allocated via api->malloc - even if the app leaks or is killed. We prove it
by reading `free` after each of several launches and asserting the free-heap
figure is identical every time (a leak would make it shrink monotonically).

RAMFS app images are transient: a successful launch deletes its source after the
ELF has been relocated. Re-upload before every launch and compare the resting
heap after each run. Each launch and each `free` is its own CLI session because
commands following a streaming launch could otherwise become app input.
"""

import argparse
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port, build_app,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
APP = "hello"
RAMFS_PATH = "/ramfs/hello"
LAUNCHES = 4


def cli(cdc_port, script, timeout=90):
    r = subprocess.run(
        [CLI_BIN, cdc_port],
        input=script,
        capture_output=True, text=True, timeout=timeout,
    )
    return r, re.sub(r'\033\[[0-9;]*m', '', r.stdout)


def read_free(cdc_port):
    """Return free-heap bytes reported by the device's `free` command, or None."""
    _, out = cli(cdc_port, "free\nexit\n", timeout=40)
    m = re.search(r'heap:\s*(\d+)/', out)
    return int(m.group(1)) if m else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--platform", required=True)
    args = parser.parse_args()
    platform = args.platform

    if platform not in PLATFORMS:
        print(f"FAIL: unknown platform {platform}")
        return 1
    if not os.path.isfile(CLI_BIN):
        print(f"SKIP: CLI binary not found at {CLI_BIN}")
        return 77

    elf = build_app(REPO_ROOT, APP, platform)
    if not elf:
        print(f"SKIP: could not build {APP} for {platform} (toolchain missing?)")
        return 77

    usb_dev = find_usb_device(USB_VID, USB_PID)
    if not usb_dev:
        print("FAIL: Fantasi device not found")
        return 1
    cdc_port = find_cdc_port(usb_dev)
    if not cdc_port:
        print("FAIL: CDC port not found")
        return 1
    print(f"  CDC: {cdc_port}  app: {os.path.relpath(elf, REPO_ROOT)}")

    # Remove a source left by an interrupted earlier run. A successful launch
    # consumes it, so each measured iteration provisions a fresh copy.
    cli(cdc_port, f"rm {RAMFS_PATH}\nexit\n", timeout=60)
    frees = []
    for i in range(LAUNCHES):
        r, out = cli(cdc_port, f"upload {elf} {RAMFS_PATH}\nexit\n")
        if RAMFS_PATH not in out and "hello" not in out:
            print(f"FAIL: upload {i + 1} did not report success")
            print(f"  stdout: {r.stdout[:400]}")
            return 1
        _, out = cli(cdc_port, f"launch {RAMFS_PATH}\n", timeout=60)
        if "exit 42" not in out:
            print(f"FAIL: launch {i + 1} did not run to completion")
            print(f"  stdout: {out[:400]}")
            cli(cdc_port, f"rm {RAMFS_PATH}\nexit\n", timeout=60)
            return 1
        f = read_free(cdc_port)
        if f is None:
            print(f"FAIL: could not read free after launch {i + 1}")
            cli(cdc_port, f"rm {RAMFS_PATH}\nexit\n", timeout=60)
            return 1
        frees.append(f)
        print(f"  after launch {i + 1}: {f} B free")

    cli(cdc_port, f"rm {RAMFS_PATH}\nexit\n", timeout=60)

    # No leak: every post-launch free must be identical.
    if len(set(frees)) != 1:
        print(f"FAIL: heap leak across launches: {frees}")
        print(f"  net drift over {LAUNCHES} launches: {frees[0] - frees[-1]} B")
        return 1

    # The first launch may lazily create process-wide app infrastructure. It is
    # intentionally retained, so the leak invariant is equality across the
    # post-launch resting states rather than equality with a pre-first-use value.
    print(f"  free stable at {frees[0]} B across {LAUNCHES} launches - no leak")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
