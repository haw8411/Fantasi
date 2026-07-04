#!/usr/bin/env python3
"""Verify directory operations on the device's persistent storage.

Flow (all via the host CLI over USB/MSC, so it runs the same on every platform):
  1. mkdir a fresh directory.
  2. upload a file into it, then read it back (cross-session) and check the bytes.
  3. confirm the directory + file are listed.
  4. rm the file, rmdir the directory.
  5. assert the directory is gone - not listed, and the file is unreadable.

Exercises the synthetic-FAT subdirectory path → VFS → LittleFS mkdir/create/
remove, end to end.
"""

import argparse
import os
import re
import subprocess
import sys
import uuid

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
DIRNAME = "/dtest"
FILENAME = "/dtest/hello.txt"


def cli(cdc_port, script, timeout=90):
    """Run a CLI session; return combined (stdout+stderr), ANSI-stripped.
    'cannot open' / errors go to stderr, so we need both to assert deletion."""
    r = subprocess.run(
        [CLI_BIN, cdc_port],
        input=script,
        capture_output=True, text=True, timeout=timeout,
    )
    return re.sub(r'\033\[[0-9;]*m', '', r.stdout + r.stderr)


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

    content = f"dir-test-{uuid.uuid4()}\n"
    local = os.path.join(REPO_ROOT, "build", "dir_ops.txt")
    with open(local, "w") as f:
        f.write(content)

    # Clean any leftover from a previous run (best effort).
    cli(cdc_port, f"rm {FILENAME}\nrmdir {DIRNAME}\nexit\n", timeout=60)

    # 1 + 2. Create the directory and a file inside it.
    print(f"  [mkdir {DIRNAME} + upload {FILENAME}]")
    out = cli(cdc_port, f"mkdir {DIRNAME}\nupload {local} {FILENAME}\nexit\n")
    if FILENAME not in out and "hello.txt" not in out:
        print("FAIL: mkdir/upload did not report success")
        print(f"  output: {out[:400]}")
        os.remove(local); return 1

    # 3. Read it back (fresh session) and confirm it's listed + correct.
    print(f"  [ls {DIRNAME} + cat {FILENAME}]")
    out = cli(cdc_port, f"ls {DIRNAME}\ncat {FILENAME}\nexit\n")
    if "hello.txt" not in out:
        print(f"FAIL: file not listed in {DIRNAME}")
        print(f"  output: {out[:400]}")
        os.remove(local); return 1
    if content.strip() not in out:
        print("FAIL: file content mismatch when read from the directory")
        print(f"  expected: {content.strip()!r}")
        print(f"  output: {out[:400]}")
        os.remove(local); return 1
    print("  directory + file created and read back correctly")

    # 4. Delete the file then the directory.
    print(f"  [rm {FILENAME} + rmdir {DIRNAME}]")
    cli(cdc_port, f"rm {FILENAME}\nrmdir {DIRNAME}\nexit\n")

    # 5. Assert the directory is gone: not listed, and the file is unreadable.
    out = cli(cdc_port, f"ls /\ncat {FILENAME}\nexit\n")
    os.remove(local)
    # `ls /` lists bare names ("dtest/"); make sure ours isn't among them.
    listed = re.search(r'(^|\s)dtest/', out)
    readable = content.strip() in out
    if listed or readable:
        print(f"FAIL: directory still present after rmdir (listed={bool(listed)}, "
              f"file_readable={readable})")
        print(f"  output: {out[:400]}")
        return 1
    print(f"  {DIRNAME} successfully deleted (not listed, file unreadable)")

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
