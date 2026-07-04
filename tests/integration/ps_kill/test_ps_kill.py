#!/usr/bin/env python3
"""Verify the `ps` task list and cross-channel `kill`.

`ps` reports every FreeRTOS task (state, priority, free stack) plus a heap
summary. `kill` stops the running app - and because a launched app owns the
channel it runs on (that session streams the app until it exits or is ^C'd), a
kill must arrive on a DIFFERENT channel. This test:

  1. checks `ps` lists tasks + heap (all platforms), and
  2. launches the spinning `spin` app over CDC serial, then over the WebUSB vendor
     pipe confirms `ps` now lists the app task and `kill` stops it - the serial
     session ends with "[killed]" and the app's memory returns to baseline.

Cross-channel needs a composite device (concurrent CDC + WebUSB). Switch-mode
devices (PM3) have only one active channel, so they SKIP part 2.
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
    find_usb_device, find_cdc_port, ensure_cdc, webusb_send, build_app,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
APP = "spin"
RAMFS_PATH = "/ramfs/spin"


def strip_ansi(s):
    return re.sub(r'\033\[[0-9;]*m', '', s)


def cli(script, port, timeout=40):
    r = subprocess.run([CLI_BIN, port], input=script,
                       capture_output=True, text=True, timeout=timeout)
    return strip_ansi(r.stdout)


def read_free(port):
    m = re.search(r'heap:\s*(\d+)/', cli("free\nexit\n", port, timeout=40))
    return int(m.group(1)) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", required=True)
    platform = ap.parse_args().platform

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
    if not find_usb_device(USB_VID, USB_PID):
        print("FAIL: Fantasi device not found")
        return 1

    ensure_cdc()
    cdc = find_cdc_port()
    if not cdc or "ttyACM" not in cdc:
        print("FAIL: CDC port not found")
        return 1
    print(f"  CDC: {cdc}")

    # --- part 1: ps lists tasks + heap (every platform) ---
    ps = cli("ps\nexit\n", cdc)
    if "PID" not in ps or "NAME" not in ps or "cli" not in ps or "heap:" not in ps:
        print("FAIL: ps output malformed")
        print(f"  stdout: {ps[:400]!r}")
        return 1
    print("  ps: task list + heap OK")

    # --- part 2: cross-channel kill (composite devices only) ---
    if PLATFORMS[platform]["msc_mode"] != "composite":
        print("  SKIP: cross-channel kill needs a composite device "
              "(switch-mode has one active channel)")
        return 77

    cli(f"upload {elf} {RAMFS_PATH}\nexit\n", cdc)
    baseline = read_free(cdc)
    if baseline is None:
        print("FAIL: could not read baseline free")
        return 1
    print(f"  baseline free (spin resident): {baseline} B")

    # Channel A (CDC serial): launch spin and let it stream.
    print("  [launch spin over CDC, kill from WebUSB]")
    a = subprocess.Popen([CLI_BIN, cdc], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, bufsize=1)
    ended = True
    try:
        a.stdin.write(f"launch {RAMFS_PATH}\n"); a.stdin.flush()
        time.sleep(3.0)   # let spin tick over CDC

        # Channel B (WebUSB): ps must now show the app task, then kill it.
        ps2 = webusb_send("ps", read_secs=1.5)
        if b"app" not in ps2:
            print("FAIL: `ps` over WebUSB did not list the running app task")
            print(f"  raw: {ps2[:200]!r}")
            a.kill(); a.communicate(); return 1
        killresp = webusb_send("kill", read_secs=1.5)
        if b"stopping app" not in killresp:
            print("FAIL: `kill` over WebUSB did not confirm")
            print(f"  raw: {killresp[:200]!r}")
            a.kill(); a.communicate(); return 1

        time.sleep(1.0)
        try:
            a.stdin.write("exit\n"); a.stdin.flush()
        except (BrokenPipeError, OSError):
            pass
        aout, _ = a.communicate(timeout=20)
    except subprocess.TimeoutExpired:
        a.kill(); aout, _ = a.communicate(); ended = False
    aout = strip_ansi(aout)

    if not ended:
        print("FAIL: CDC launch session never ended after cross-channel kill")
        print(f"  tail: {aout[-400:]!r}")
        return 1
    if "tick" not in aout:
        print("FAIL: spin produced no output - did it run?")
        return 1
    if "[killed]" not in aout:
        print("FAIL: launch channel did not report [killed]")
        print(f"  tail: {aout[-400:]!r}")
        return 1
    print("  spin killed from the other channel (CDC session ended with [killed])")

    ensure_cdc()
    after = read_free(cdc)
    cli(f"rm {RAMFS_PATH}\nexit\n", cdc)
    if after is None:
        print("FAIL: device unresponsive after kill")
        return 1
    if after != baseline:
        print(f"FAIL: kill did not fully reclaim - {baseline - after} B unreturned")
        return 1

    print("  memory reclaimed (free back to baseline)")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
