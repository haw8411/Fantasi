#!/usr/bin/env python3
"""Verify the firmware loads and runs the reference 'hello' app end-to-end.

Builds the per-architecture hello ELF (Cortex-M4 for FZ/CU, ARM7TDMI for PM3),
uploads it to /ramfs, and `launch`es it - checking that the relocated app
actually executes on the device: it prints its greeting, exercises the API
heap (api->malloc/free), reports the ABI version, and returns its exit code.

This is the real proof the ELF loader works on the target: a Cortex-M ELF on an
ARM7TDMI (or vice-versa) would fault rather than print 'exit 42'.
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

# What apps/hello/hello.c is expected to emit (see that file).
EXPECT_GREETING = "hello from a Fantasi app"
EXPECT_HEAP = "x" * 31          # the malloc'd buffer it fills and prints
EXPECT_ABI = "abi=1"
EXPECT_EXIT = "exit 42"


def cli(cdc_port, script, timeout=90):
    r = subprocess.run(
        [CLI_BIN, cdc_port],
        input=script,
        capture_output=True, text=True, timeout=timeout,
    )
    return r, re.sub(r'\033\[[0-9;]*m', '', r.stdout)


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

    # Upload the app to /ramfs (its own session - RAM survives between sessions).
    print("  [Uploading hello to /ramfs]")
    r, out = cli(cdc_port, f"upload {elf} {RAMFS_PATH}\nexit\n")
    if RAMFS_PATH not in out and "hello" not in out:
        print("FAIL: upload did not report success")
        print(f"  stdout: {r.stdout[:400]}")
        return 1

    # Launch it. `launch` streams the app's output; keep it the last line so the
    # streaming session doesn't swallow following commands as app input.
    print("  [Launching]")
    r, out = cli(cdc_port, f"launch {RAMFS_PATH}\n", timeout=60)

    missing = [s for s in (EXPECT_GREETING, EXPECT_HEAP, EXPECT_ABI, EXPECT_EXIT)
               if s not in out]
    if missing:
        print(f"FAIL: launch output missing: {missing}")
        print(f"  stdout: {out[:600]}")
        # best-effort cleanup before returning
        cli(cdc_port, f"rm {RAMFS_PATH}\nexit\n", timeout=60)
        return 1

    print(f"  greeting, heap buffer, {EXPECT_ABI}, {EXPECT_EXIT} - all present")

    # Cleanup.
    cli(cdc_port, f"rm {RAMFS_PATH}\nexit\n", timeout=60)

    print("  hello app loaded, ran, and exited correctly")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
