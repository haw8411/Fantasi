#!/usr/bin/env python3
"""Verify the firmware runs a Berry script through the `launch` command.

Uploads berry_test.be to /ramfs and `launch`es it. `launch` dispatches a .be
path to the embedded Berry VM (be_exec), which runs in the launching CLI task -
so the script's print() output streams straight back to us. The script exercises
integer arithmetic, string ops, and the VFS-backed Berry file port (open/write/
read under /ramfs), printing tagged tokens we assert on.

Needs a Berry-enabled build (Flipper/Kiisu/Chameleon/Proxmark3); on a firmware
without Berry, `launch *.be` falls through to the ELF loader and fails to load,
which we treat as SKIP.
"""

import argparse
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
SCRIPT_SRC = os.path.join(os.path.dirname(__file__), "berry_test.be")
SCRIPT_PATH = "/ramfs/berry_test.be"
DATA_PATH = "/ramfs/berry_launch.txt"   # written by the script; cleaned up after

# What berry_test.be is expected to print (see that file).
EXPECT = ("BERRY_SUM 55", "BERRY_STR ababab", "BERRY_FILE berry-vfs-55", "BERRY_DONE")


def cli(cdc_port, script, timeout=90):
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
    if not os.path.isfile(CLI_BIN):
        print(f"SKIP: CLI binary not found at {CLI_BIN}")
        return 77

    usb_dev = find_usb_device(USB_VID, USB_PID)
    if not usb_dev:
        print("FAIL: Fantasi device not found")
        return 1
    cdc_port = find_cdc_port(usb_dev)
    if not cdc_port:
        print("FAIL: CDC port not found")
        return 1
    print(f"  CDC: {cdc_port}")

    # Upload the script to /ramfs (RAM survives between CLI sessions).
    print("  [Uploading berry_test.be to /ramfs]")
    out = cli(cdc_port, f"upload {SCRIPT_SRC} {SCRIPT_PATH}\nexit\n")
    if SCRIPT_PATH not in out and "berry_test" not in out:
        print("FAIL: upload did not report success")
        print(f"  stdout: {out[:400]}")
        return 1

    # Launch it. Keep `launch` the last line so its streamed output isn't cut off.
    print("  [Launching the Berry script]")
    out = cli(cdc_port, f"launch {SCRIPT_PATH}\n", timeout=60)

    # No Berry in this build -> `launch *.be` fell through to the ELF loader.
    if "load failed" in out or "not found" in out.lower():
        print("SKIP: firmware has no Berry (launch fell through to ELF loader)")
        cli(cdc_port, f"rm {SCRIPT_PATH}\nexit\n", timeout=60)
        return 77

    missing = [tok for tok in EXPECT if tok not in out]
    if missing:
        print(f"FAIL: Berry output missing: {missing}")
        print(f"  stdout: {out[:600]}")
        cli(cdc_port, f"rm {SCRIPT_PATH}\nrm {DATA_PATH}\nexit\n", timeout=60)
        return 1

    print("  arithmetic, strings, and VFS file round-trip - all present")

    # Cleanup.
    cli(cdc_port, f"rm {SCRIPT_PATH}\nrm {DATA_PATH}\nexit\n", timeout=60)

    print("  Berry script ran via launch and used the VFS file port")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
