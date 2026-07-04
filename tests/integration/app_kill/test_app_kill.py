#!/usr/bin/env python3
"""Verify Ctrl-C kills a running app and fully reclaims its memory.

The 'spin' app loops forever (printing ticks). We launch it, let it run, then
send Ctrl-C (0x03) over the link - the firmware must stop its task, free the app
image / task stack / allocations, and return to the prompt. We confirm two
things: the launch session actually ends after Ctrl-C (a failed kill would hang
the streaming session), and `free` returns to the pre-launch baseline (the kill
path reclaims everything, not just the clean-exit path).

Unlike the other app tests this drives the CLI interactively (Popen): the Ctrl-C
has to arrive *while* the app is running, which a single piped script can't time.
"""

import argparse
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port, build_app,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
APP = "spin"
RAMFS_PATH = "/ramfs/spin"


def strip_ansi(s):
    return re.sub(r'\033\[[0-9;]*m', '', s)


def cli(cdc_port, script, timeout=60):
    r = subprocess.run([CLI_BIN, cdc_port], input=script,
                       capture_output=True, text=True, timeout=timeout)
    return r, strip_ansi(r.stdout)


def read_free(cdc_port):
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

    # Upload spin, then capture the pre-launch baseline (app resident, idle).
    r, out = cli(cdc_port, f"upload {elf} {RAMFS_PATH}\nexit\n")
    if RAMFS_PATH not in out and "spin" not in out:
        print("FAIL: upload did not report success")
        print(f"  stdout: {r.stdout[:400]}")
        return 1

    baseline = read_free(cdc_port)
    if baseline is None:
        print("FAIL: could not read baseline free")
        return 1
    print(f"  baseline free (spin resident, not running): {baseline} B")

    # Launch interactively: let it tick, then send Ctrl-C while it runs.
    print("  [Launching spin, then sending Ctrl-C]")
    p = subprocess.Popen([CLI_BIN, cdc_port], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, bufsize=1)
    killed_cleanly = True
    try:
        p.stdin.write(f"launch {RAMFS_PATH}\n"); p.stdin.flush()
        time.sleep(2.5)                       # let it spin/tick a while
        p.stdin.write("\x03"); p.stdin.flush()  # Ctrl-C
        time.sleep(0.5)
        try:
            p.stdin.write("exit\n"); p.stdin.flush()
        except (BrokenPipeError, OSError):
            pass
        out, _ = p.communicate(timeout=20)
    except subprocess.TimeoutExpired:
        # The session never ended → Ctrl-C did not stop the app.
        p.kill()
        out, _ = p.communicate()
        killed_cleanly = False
    out = strip_ansi(out)

    if not killed_cleanly:
        print("FAIL: launch session did not end after Ctrl-C")
        print(f"  stdout tail: {out[-400:]}")
        return 1
    if "tick" not in out:
        print("FAIL: spin produced no output - did it run?")
        print(f"  stdout: {out[:400]}")
        return 1
    ticks = out.count("tick")
    print(f"  spin ran ({ticks} ticks seen), then Ctrl-C sent")
    # Note: the host CLI stops reading the moment it forwards 0x03, so the
    # device's "^C aborted" line isn't captured here - the proof the kill worked
    # is below: the device is responsive again AND the heap is fully reclaimed.
    # (A failed kill would leave the CLI task stuck in the stream loop, so the
    #  following `free` would get no response and read_free() would return None.)

    after = read_free(cdc_port)
    if after is None:
        print("FAIL: device unresponsive after Ctrl-C (kill did not return to CLI)")
        return 1
    print(f"  device responsive after kill; free: {after} B (baseline {baseline})")

    cli(cdc_port, f"rm {RAMFS_PATH}\nexit\n", timeout=60)

    if after != baseline:
        print(f"FAIL: kill did not fully reclaim - {baseline - after} B unreturned")
        return 1

    print("  Ctrl-C stopped the app and reclaimed all its memory")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
