#!/usr/bin/env python3
"""Verify the host CLI can upload a file and the firmware can read it back.

Uploads a unique test file with the CLI's ``upload`` command, reads it back
with ``cat``, and checks byte-for-byte equality. The CLI handles MSC-mode
switching and the FAT mount internally, so this test is identical across
composite (FZ, CU) and switch-mode (PM3) platforms - only the CDC serial port
is passed.
"""

import argparse
import os
import subprocess
import re
import sys
import time
import uuid

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")


def step(msg):
    print(f"  [{msg}]")


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

    if args.platform not in PLATFORMS:
        print(f"FAIL: unknown platform {args.platform}")
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

    # Create a unique test file with known content.
    test_content = f"fantasi-test-{uuid.uuid4()}\n"
    test_file = os.path.join(REPO_ROOT, "build", "cli_upload_test.txt")
    with open(test_file, "w") as f:
        f.write(test_content)

    # Upload via CLI.
    step("Uploading test file via CLI")
    r, out = cli(cdc_port, f"upload {test_file} /test.txt\nexit\n")
    if "test.txt" not in out:
        print("FAIL: upload did not succeed")
        print(f"  stdout: {r.stdout[:500]}")
        print(f"  stderr: {r.stderr[:500]}")
        return 1
    print("  Upload OK")

    # Read back via CLI cat (re-resolve the port in case of an MSC cycle).
    time.sleep(3)
    cdc_port = find_cdc_port() or cdc_port
    step("Reading back via CLI cat")
    r, clean = cli(cdc_port, "cat /test.txt\nexit\n")

    if test_content.strip() not in clean:
        print("FAIL: content mismatch")
        print(f"  expected: {test_content.strip()!r}")
        print(f"  raw stdout ({len(r.stdout)} bytes): {r.stdout[:500]!r}")
        print(f"  stderr: {r.stderr[:500]!r}")
        return 1
    print("  Content verified")

    # Clean up.
    time.sleep(2)
    cdc_port = find_cdc_port() or cdc_port
    cli(cdc_port, "rm /test.txt\nexit\n")
    os.remove(test_file)

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
