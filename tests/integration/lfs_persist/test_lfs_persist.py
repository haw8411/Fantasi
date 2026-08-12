#!/usr/bin/env python3
"""Verify that internal-flash (LittleFS) contents survive a firmware update.

Writes a sentinel file to the device's filesystem, runs the real
``make PLATFORM=<x> flash`` flow, and checks that the sentinel is still
present and byte-identical after reboot.

Everything goes through the host CLI (``build/cli/fantasi``), which talks to
the device over its CDC serial / MSC FAT path. The CLI handles MSC-mode
switching internally, so this test is identical across composite (FZ, CU) and
switch-mode (PM3) platforms - no littlefs-python, no raw block access, no
on-disk-format coupling.
"""

import argparse
import os
import re
import subprocess
import sys
import time
import uuid

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port, ensure_cdc,
    wait_for_usb,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
SENTINEL_PATH = "/test_persist"


def step(msg):
    print(f"  [{msg}]")


def cli(cdc_port, script, timeout=90):
    """Run a CLI session, feeding `script` on stdin. Returns stdout (ANSI-stripped)."""
    r = subprocess.run(
        [CLI_BIN, cdc_port],
        input=script,
        capture_output=True, text=True, timeout=timeout,
    )
    return re.sub(r'\033\[[0-9;]*m', '', r.stdout)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--platform", required=True)
    args = parser.parse_args()
    platform = args.platform

    if platform not in PLATFORMS:
        print(f"FAIL: unknown platform {platform}")
        return 1

    if PLATFORMS[platform].get("no_auto_dfu_return"):
        print(f"SKIP: {platform} cannot re-enter its app after a USB DFU flash "
              "without a manual power-cycle (AT32 ROM DFU + PB0 power-latch), so "
              "the unattended reflash-and-verify cycle can't run")
        return 77

    if not os.path.isfile(CLI_BIN):
        print(f"SKIP: CLI binary not found at {CLI_BIN}")
        return 77

    sentinel = f"persist-{uuid.uuid4()}\n"
    sentinel_file = os.path.join(REPO_ROOT, "build", "lfs_persist_sentinel.txt")
    with open(sentinel_file, "w") as f:
        f.write(sentinel)

    usb_dev = find_usb_device(USB_VID, USB_PID)
    if not usb_dev:
        print("FAIL: Fantasi device not found on USB")
        return 1

    cdc_port = find_cdc_port(usb_dev)
    if not cdc_port:
        print("FAIL: CDC port not found")
        return 1
    print(f"  CDC: {cdc_port}")

    # Upload the sentinel via the CLI.
    step("Writing sentinel via CLI upload")
    out = cli(cdc_port, f"upload {sentinel_file} {SENTINEL_PATH}\nexit\n")
    if SENTINEL_PATH not in out:
        print("FAIL: upload failed")
        print(f"  stdout: {out[:300]}")
        return 1
    print(f"  Wrote sentinel: {sentinel.strip()!r}")

    # Verify pre-flash.
    step("Verifying sentinel (pre-flash)")
    time.sleep(4)
    cdc_port = find_cdc_port() or cdc_port
    out = cli(cdc_port, f"cat {SENTINEL_PATH}\nexit\n")
    if sentinel.strip() not in out:
        print("FAIL: sentinel not readable before flash")
        print(f"  stdout: {out[:300]}")
        return 1
    print("  Sentinel verified")

    # Flash. `make flash` needs the CDC serial to send the `dfu` command, but a
    # switch-mode device (PM3) whose CLI auto-upgraded is in WebUSB with no CDC -
    # return it to serial first.
    step(f"Running: make PLATFORM={platform} flash")
    ensure_cdc()
    time.sleep(3)
    r = subprocess.run(["make", f"PLATFORM={platform}", "flash"], cwd=REPO_ROOT)
    if r.returncode != 0:
        print(f"FAIL: make flash exited {r.returncode}")
        return 1
    print("  Flash complete")

    # Wait for re-enumeration.
    step("Waiting for Fantasi to reboot")
    time.sleep(4)
    usb_dev2 = wait_for_usb(USB_VID, USB_PID)
    if not usb_dev2:
        print("FAIL: device did not re-enumerate after flash")
        return 1

    cdc_port2 = find_cdc_port(usb_dev2)
    for _ in range(10):
        if cdc_port2:
            break
        time.sleep(0.5)
        cdc_port2 = find_cdc_port(usb_dev2)
    if not cdc_port2:
        print("FAIL: CDC port not found after flash")
        return 1

    # Verify post-flash. The CDC port can re-appear a moment before the storage
    # subsystem is ready to MSC-mount, so retry the read a few times before
    # declaring failure (the data itself is committed to flash by this point).
    step("Verifying sentinel after flash")
    out = ""
    for _ in range(10):
        out = cli(cdc_port2, f"cat {SENTINEL_PATH}\nexit\n")
        if sentinel.strip() in out:
            break
        time.sleep(1)
        cdc_port2 = find_cdc_port() or cdc_port2
    if sentinel.strip() not in out:
        print("FAIL: sentinel not found after flash")
        print(f"  expected: {sentinel.strip()!r}")
        print(f"  stdout: {out[:500]}")
        return 1

    # Cleanup.
    time.sleep(3)
    cdc_port2 = find_cdc_port() or cdc_port2
    cli(cdc_port2, f"rm {SENTINEL_PATH}\nexit\n")
    os.remove(sentinel_file)

    print("  Sentinel intact - internal flash survived the firmware update")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
