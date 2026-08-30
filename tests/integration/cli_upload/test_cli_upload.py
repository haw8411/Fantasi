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
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port, ensure_cdc,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")


def step(msg):
    print(f"  [{msg}]")


def cli(cdc_port, script, timeout=90):
    one_shot = ";".join(line.strip() for line in script.splitlines()
                        if line.strip())
    r = subprocess.run(
        # Pin the legacy serial/MSC route. Merely supplying a CDC path is not
        # enough: the production CLI normally upgrades that connection to
        # WebUSB, whose protobuf file path would bypass the FAT code under test.
        # Use -c rather than piped stdin: serial response streaming intentionally
        # watches stdin for cancellable/app input and could consume a queued test
        # script while the startup settings query is still completing.
        [CLI_BIN, "--serial", cdc_port, "-c", one_shot],
        capture_output=True, text=True, timeout=timeout,
    )
    return r, re.sub(r'\033\[[0-9;]*m', '', r.stdout)


def usb_cli(script, timeout=90):
    """Read device-side VFS truth, bypassing the mounted FAT page cache."""
    commands = [line.strip() for line in script.splitlines() if line.strip()]
    r = subprocess.run(
        [CLI_BIN, "--usb"], input="\n".join(commands) + "\nexit\n",
        capture_output=True, text=True, timeout=timeout,
    )
    return r, re.sub(r'\033\[[0-9;]*m', '', r.stdout + r.stderr)


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

    # This test specifically covers the mounted-FAT/MSC route. The PM3 test
    # suite normally remains in WebUSB mode between cases, which would silently
    # select protobuf upload and miss the switch-mode transport entirely.
    if args.platform == "proxmark3" and not ensure_cdc():
        print("FAIL: could not return switch-mode device to CDC")
        return 1
    cdc_port = find_cdc_port()
    if not cdc_port:
        print("FAIL: CDC port not found")
        return 1
    print(f"  CDC: {cdc_port}")

    # Exercise a tiny text file and a larger binary file. CRC checks every byte
    # of the latter without dumping it over the CLI.
    test_content = f"fantasi-test-{uuid.uuid4()}\n"
    test_file = os.path.join(REPO_ROOT, "build", "cli_upload_test.txt")
    with open(test_file, "w") as f:
        f.write(test_content)
    dot_file = os.path.join(REPO_ROOT, "build", "cli_upload_dot.txt")
    with open(dot_file, "w") as f:
        f.write(test_content)
    large_content = bytes(((i * 131 + 17) & 0xff) for i in range(12990))
    large_file = os.path.join(REPO_ROOT, "build", "cli_upload_large.bin")
    with open(large_file, "wb") as f:
        f.write(large_content)
    # A distinct same-length replacement catches incremental-state that
    # mistakenly preserves a previously committed prefix.
    replacement_content = bytes(((i * 73 + 91) & 0xff) for i in range(12990))
    replacement_file = os.path.join(REPO_ROOT, "build", "cli_upload_replacement.bin")
    with open(replacement_file, "wb") as f:
        f.write(replacement_content)
    replacement_crc = zlib.crc32(replacement_content) & 0xffffffff

    # Upload via CLI.
    ramfs_name = os.path.basename(test_file)
    ramfs_path = f"/ramfs/{ramfs_name}"
    dot_path = f"/ramfs/{os.path.basename(dot_file)}"
    step("Uploading through explicit, omitted, and dot directory targets via MSC")
    r, out = cli(
        cdc_port,
        f"upload {test_file} /test.txt\n"
        f"upload {large_file} /test-large.bin\n"
        f"upload {replacement_file} /test-large.bin\n"
        f"cd /ramfs\n"
        f"upload {test_file}\n"
        f"upload {dot_file} .\n"
        f"cd /\n"
        "exit\n",
        timeout=240,
    )
    errors = (r.stdout + r.stderr).lower()
    if ("test.txt" not in out or ramfs_path not in out or dot_path not in out or
            "test-large.bin" not in out or
            "sync failed" in errors or "write failed" in errors or
            "not committed" in errors):
        print("FAIL: upload did not succeed")
        print(f"  stdout: {r.stdout[:500]}")
        print(f"  stderr: {r.stderr[:500]}")
        return 1
    print("  All uploads OK")

    # Read back via CLI cat (re-resolve the port in case of an MSC cycle).
    time.sleep(3)
    cdc_port = find_cdc_port() or cdc_port
    step("Reading back via CLI cat")
    r, clean = cli(cdc_port,
                   f"cat /test.txt\ncat {ramfs_path}\ncat {dot_path}\nexit\n")

    if clean.count(test_content.strip()) < 3:
        print("FAIL: content mismatch")
        print(f"  expected: {test_content.strip()!r}")
        print(f"  raw stdout ({len(r.stdout)} bytes): {r.stdout[:500]!r}")
        print(f"  stderr: {r.stderr[:500]!r}")
        return 1
    print("  Content verified")

    time.sleep(2)
    cdc_port = find_cdc_port() or cdc_port
    step("Verifying large-file length and CRC")
    r, clean = cli(cdc_port, "crc32 /test-large.bin\nexit\n", timeout=120)
    expected_crc = f"{replacement_crc:08x} {len(replacement_content)} /test-large.bin"
    if expected_crc not in clean:
        print("FAIL: large-file CRC mismatch")
        print(f"  expected: {expected_crc!r}")
        print(f"  stdout: {r.stdout[:500]!r}")
        print(f"  stderr: {r.stderr[:500]!r}")
        return 1
    print("  Large-file CRC verified")

    # The large fixture has served its purpose. Release it before the separate
    # reconcile stress so the PM3's 128 KiB internal filesystem has working room.
    cli(cdc_port, "rm /test-large.bin\nexit\n", timeout=120)

    # Composite targets keep their MSC volume mounted indefinitely. This exceeds
    # both bounded reconcile tables in one mount and also fills a new directory's
    # initial FAT cluster, catching leaked completed records, moving directory
    # boundaries, and unreleased LFN reconstruction slots. Verify over WebUSB so
    # Linux's mounted FAT cache cannot hide a device-side wrong/missing pathname.
    stress_tag = uuid.uuid4().hex[:8]
    stress_dir = f"/msc_recycle_{stress_tag}"
    stress_names = [f"long_pending_{i:02d}.txt" for i in range(20)]
    stress_script = [f"mkdir {stress_dir}"]
    stress_script.extend(
        f"upload {test_file} {stress_dir}/{name}" for name in stress_names
    )
    step("Recycling MSC write/LFN state across 20 files in one mount")
    r, out = cli(cdc_port, "\n".join(stress_script) + "\nexit\n", timeout=300)
    errors = (r.stdout + r.stderr).lower()
    if ("sync failed" in errors or "write failed" in errors or
            "not committed" in errors or "i/o error" in errors):
        print("FAIL: long-lived MSC upload sequence failed")
        print(f"  stdout: {r.stdout[-1000:]}")
        print(f"  stderr: {r.stderr[-1000:]}")
        return 1

    r, truth = usb_cli(f"ls {stress_dir}\n", timeout=120)
    missing = [name for name in stress_names if name not in truth]
    if r.returncode or missing:
        print(f"FAIL: MSC reconcile/LFN paths missing after 20 files: {missing}")
        print(f"  output: {truth[:2000]!r}")
        return 1
    tiny_crc = zlib.crc32(test_content.encode()) & 0xffffffff
    probes = (stress_names[0], stress_names[11], stress_names[-1])
    _, truth = usb_cli("\n".join(
        f"crc32 {stress_dir}/{name}" for name in probes
    ), timeout=120)
    if truth.count(f"{tiny_crc:08x} {len(test_content)}") != len(probes):
        print("FAIL: recycled MSC upload content mismatch")
        print(f"  output: {truth[:1000]!r}")
        return 1
    print("  20-file reconcile, directory extension, and LFN reuse verified")

    usb_cli("\n".join(f"rm {stress_dir}/{name}" for name in stress_names) +
            f"\nrmdir {stress_dir}\n", timeout=180)
    if args.platform == "proxmark3":
        ensure_cdc()

    # Clean up.
    time.sleep(2)
    cdc_port = find_cdc_port() or cdc_port
    cli(cdc_port,
        f"rm /test.txt\nrm {ramfs_path}\nrm {dot_path}\nrm /test-large.bin\nexit\n",
        timeout=120)
    os.remove(test_file)
    os.remove(dot_file)
    os.remove(large_file)
    os.remove(replacement_file)

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
