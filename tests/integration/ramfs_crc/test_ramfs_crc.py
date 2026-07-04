#!/usr/bin/env python3
"""Verify the RAM-backed /ramfs round-trips arbitrary content byte-for-byte.

Uploads a random binary blob to /ramfs via the host CLI, then asks the device
for its CRC32 (`crc32 /ramfs/<name>`) and checks it matches the host-computed
CRC32 of what was uploaded. Also confirms the file is listed and reports the
correct size. /ramfs is RAM-only, so everything happens in one CLI session (no
reboot) - the MSC switch on switch-mode platforms (PM3) does not lose RAM.

This exercises the full synthetic-FAT write path → VFS router → ramfs store and
back out through the read path, identically on every target that enables apps
(Flipper, Chameleon, Proxmark3).
"""

import argparse
import os
import re
import subprocess
import sys
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
BLOB_SIZE = 4096          # spans several FAT sectors + a cluster boundary


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

    # A deterministic-but-arbitrary blob (binary, not text) - a faithful FS must
    # return it bit-for-bit. zlib.Random-free: derive bytes from a fixed seed.
    blob = bytes((i * 73 + 19) & 0xff for i in range(BLOB_SIZE))
    want_crc = zlib.crc32(blob) & 0xffffffff
    blob_file = os.path.join(REPO_ROOT, "build", "ramfs_blob.bin")
    with open(blob_file, "wb") as f:
        f.write(blob)
    print(f"  blob: {BLOB_SIZE} bytes, host crc32 {want_crc:08x}")

    remote = "/ramfs/blob.bin"

    # Everything in ONE session: upload → list → crc32 → rm. /ramfs is RAM-only
    # (cleared on reboot), so no separate cleanup pass is needed - and a single
    # session means a single MSC switch on switch-mode platforms (PM3), which is
    # both faster and avoids leaving the device mid-cycle if interrupted.
    r, out = cli(cdc_port,
                 f"upload {blob_file} {remote}\nls /ramfs\ncrc32 {remote}\nrm {remote}\nexit\n",
                 timeout=120)
    os.remove(blob_file)

    if remote not in out and "blob.bin" not in out:
        print("FAIL: upload did not report success")
        print(f"  stdout: {r.stdout[:500]}")
        print(f"  stderr: {r.stderr[:300]}")
        return 1

    # Parse "<crc8> <size> /ramfs/blob.bin"
    m = re.search(r'([0-9a-f]{8})\s+(\d+)\s+' + re.escape(remote), out)
    if not m:
        print("FAIL: crc32 output not found")
        print(f"  stdout: {out[:500]}")
        return 1

    got_crc = int(m.group(1), 16)
    got_size = int(m.group(2))

    if got_size != BLOB_SIZE:
        print(f"FAIL: size mismatch - device {got_size}, expected {BLOB_SIZE}")
        return 1
    if got_crc != want_crc:
        print(f"FAIL: CRC32 mismatch - device {got_crc:08x}, expected {want_crc:08x}")
        return 1

    print(f"  device crc32 {got_crc:08x}, size {got_size} - matches")
    print("  /ramfs round-trip verified")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
