#!/usr/bin/env python3
"""Verify the device clock advances one second per real host second.

Berry's time.clock() returns clock() / CLOCKS_PER_SEC, and picolibc's clock()
sums the tms fields that core/libc_glue.c's times() fills. times() must convert
FreeRTOS ticks into CLOCKS_PER_SEC units; a wrong scale makes time.clock() run
fast or slow, and a scale that leaves a zero divisor traps and resets the device.

We sample time.clock() twice across a known host-side delay and check the device
delta matches the host delta ~1:1. A reset in times() shows up as missing output
on the first sample (-> FAIL), which no other test exercises.

Needs a Berry build with the time module; otherwise SKIP.
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
    find_usb_device, find_cdc_port,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
SCRIPT_SRC = os.path.join(os.path.dirname(__file__), "clock_rate.be")
SCRIPT_PATH = "/ramfs/clock_rate.be"

GAP_SECONDS = 6.0   # host-side delay between the two samples


def cli(cdc_port, script, timeout=30):
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

    out = cli(cdc_port, f"upload {SCRIPT_SRC} {SCRIPT_PATH}\nexit\n")
    # Success line differs by transport: CDC/FAT prints "-> path (N bytes)",
    # WebUSB chunked prints progress ending at "N / N bytes". Accept either.
    if not (re.search(r'->.*\(\d+ bytes\)', out) or re.search(r'(\d+) / \1 bytes', out)):
        print("FAIL: upload did not report success")
        print(f"  stdout: {out[:400]}")
        return 1

    def sample():
        """Launch the probe, return (device_clock_seconds, host_time) or (None, host_time)."""
        h = time.time()
        o = cli(cdc_port, f"launch {SCRIPT_PATH}\n", timeout=20)
        if "load failed" in o:
            return "no-berry", h
        m = re.search(r'CLK=([0-9.]+)', o)
        return (float(m.group(1)) if m else None), h

    d1, h1 = sample()
    if d1 == "no-berry":
        print("SKIP: firmware has no Berry / time module")
        cli(cdc_port, f"rm {SCRIPT_PATH}\nexit\n")
        return 77
    if d1 is None:
        print("FAIL: no clock output on first sample - time.clock() crashed the "
              "device (times() divide-by-zero trap?)")
        return 1

    time.sleep(GAP_SECONDS)

    d2, h2 = sample()
    if not isinstance(d2, float):
        print("FAIL: no clock output on second sample")
        return 1

    dev_delta = d2 - d1
    host_delta = h2 - h1
    print(f"  device clock delta = {dev_delta:.2f}s   host delta = {host_delta:.2f}s")

    if dev_delta <= 0:
        print("FAIL: device clock did not advance (frozen or reset)")
        return 1
    ratio = dev_delta / host_delta
    if not (0.6 < ratio < 1.6):
        print(f"FAIL: clock rate off by {ratio:.1f}x (scaling wrong in times())")
        return 1

    cli(cdc_port, f"rm {SCRIPT_PATH}\nexit\n")
    print(f"  clock advances 1:1 with host time (ratio {ratio:.2f})")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
