#!/usr/bin/env python3
"""Verify an app can drive Berry through the app API to touch the VFS.

Builds the berryvfs app (per-arch ELF), uploads it to /ramfs, and `launch`es it.
The app writes a data file, calls api->be_exec on a Berry script that reads that
file through the VFS-backed Berry file port and writes a derived result, then the
app reads the result back with api->read_file and prints it. Proves the whole
app -> API -> Berry -> VFS (read + write) chain, not just the CLI's launch path.

Needs a Berry-enabled build (Flipper/Kiisu/Chameleon/Proxmark3); if be_exec is
unavailable the app prints so and we treat it as SKIP.
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
APP = "berryvfs"
RAMFS_PATH = "/ramfs/berryvfs"

EXPECT_RESULT = "berryvfs: result=payload-2468-seen"
EXPECT_DONE = "berryvfs: done"
# Files the app leaves under /ramfs (data, script, and Berry's result).
LEFTOVERS = ("/ramfs/berryvfs_data.txt", "/ramfs/berryvfs_run.be", "/ramfs/berryvfs_out.txt")


def cli(cdc_port, script, timeout=90):
    r = subprocess.run(
        [CLI_BIN, cdc_port],
        input=script,
        capture_output=True, text=True, timeout=timeout,
    )
    return re.sub(r'\033\[[0-9;]*m', '', r.stdout)


def cleanup(cdc_port):
    rm = "".join(f"rm {p}\n" for p in (RAMFS_PATH, *LEFTOVERS))
    cli(cdc_port, rm + "exit\n", timeout=60)


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
    print(f"  app: {os.path.relpath(elf, REPO_ROOT)}")

    usb_dev = find_usb_device(USB_VID, USB_PID)
    if not usb_dev:
        print("FAIL: Fantasi device not found")
        return 1
    cdc_port = find_cdc_port(usb_dev)
    if not cdc_port:
        print("FAIL: CDC port not found")
        return 1
    print(f"  CDC: {cdc_port}")

    print("  [Uploading berryvfs to /ramfs]")
    out = cli(cdc_port, f"upload {elf} {RAMFS_PATH}\nexit\n")
    if RAMFS_PATH not in out and "berryvfs" not in out:
        print("FAIL: upload did not report success")
        print(f"  stdout: {out[:400]}")
        return 1

    print("  [Launching]")
    out = cli(cdc_port, f"launch {RAMFS_PATH}\n", timeout=60)

    if "needs firmware ABI" in out:
        print("SKIP: firmware has no Berry (be_exec unavailable)")
        cleanup(cdc_port)
        return 77

    missing = [s for s in (EXPECT_RESULT, EXPECT_DONE) if s not in out]
    if missing:
        print(f"FAIL: launch output missing: {missing}")
        print(f"  stdout: {out[:600]}")
        cleanup(cdc_port)
        return 1

    print("  app wrote data, Berry read+wrote it via the VFS, app read it back")

    cleanup(cdc_port)

    print("  app drove Berry through the API and round-tripped a VFS file")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
