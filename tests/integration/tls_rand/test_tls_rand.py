#!/usr/bin/env python3
"""Regression test for the picolibc TLS thread-pointer setup.

picolibc keeps errno, the rand()/random() state (_rand_next) and the localtime
buffer in thread-local storage, reached via __aeabi_read_tp(). Because we build
-nostartfiles, crt0's TLS init never runs; core/libc_glue.c must point the thread
pointer at a real RAM block (fantasi_tls_block). If that setup regresses, the
thread pointer is 0 and TLS variables alias the low vector table:

  * Cortex-M (Flipper/Chameleon): the rand() state write lands in the flash alias
    and is dropped, so rand() stops advancing.
  * ARM7 (Proxmark3): the write hits the SWI/abort exception vectors at 0x8, so
    the next exception hangs the device.

This exercises rand() (the cleanest TLS-backed symbol) and checks both failure
modes: that successive values advance, and that the device survives the call.

Needs a Berry-enabled build. Without Berry, `launch *.be` falls through to the
ELF loader -> SKIP.
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
SCRIPT_SRC = os.path.join(os.path.dirname(__file__), "tls_rand.be")
SCRIPT_PATH = "/ramfs/tls_rand.be"


def cli(cdc_port, script, timeout=60):
    """Run the host CLI with `script` on stdin. Raises TimeoutExpired if the
    device stops responding - which is itself a signal (ARM7 TLS hang)."""
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

    print("  [Uploading tls_rand.be to /ramfs]")
    out = cli(cdc_port, f"upload {SCRIPT_SRC} {SCRIPT_PATH}\nexit\n")
    # The CLI echoes the piped command, so SCRIPT_PATH is always in `out`; assert
    # an actual completion marker instead (failures print only to stderr, which
    # cli() discards). Two transports, two success lines: the CDC/FAT path prints
    # `<local> -> <remote> (N bytes)`, the WebUSB chunked path prints progress
    # ending at `N / N bytes`. Accept either; without this a failed upload isn't
    # noticed and the launch below is misread as "no Berry".
    uploaded = re.search(r'->.*\(\d+ bytes\)', out) or re.search(r'(\d+) / \1 bytes', out)
    if not uploaded:
        print("FAIL: upload did not report success")
        print(f"  stdout: {out[:400]}")
        return 1

    # Launch it. A TLS write to the ARM7 exception vectors hangs the device, so
    # the CLI would never return -> catch the timeout and report it as the hang.
    print("  [Launching rand() probe]")
    try:
        out = cli(cdc_port, f"launch {SCRIPT_PATH}\n", timeout=30)
    except subprocess.TimeoutExpired:
        print("FAIL: device hung during rand() - TLS thread pointer not set "
              "(ARM7 exception vectors clobbered?)")
        return 1

    # Only "load failed" means no Berry (the .be fell through to the ELF loader,
    # core/app_run.c). "not found" means the file is missing (upload failed / bad
    # path) - a real failure, so let it fall through to the checks below and FAIL
    # rather than masquerading as a skip.
    if "load failed" in out:
        print("SKIP: firmware has no Berry (launch fell through to ELF loader)")
        cli(cdc_port, f"rm {SCRIPT_PATH}\nexit\n")
        return 77

    m = re.search(r'TLS_RAND ([0-9,]+)', out)
    if not m or "TLS_DONE" not in out:
        print("FAIL: rand() probe did not complete")
        print(f"  stdout: {out[:600]}")
        cli(cdc_port, f"rm {SCRIPT_PATH}\nexit\n")
        return 1

    vals = m.group(1).split(",")
    if len(set(vals)) <= 1:
        print(f"FAIL: rand() is frozen (all values identical: {vals[0]}) - TLS "
              "state write is being dropped (thread pointer not in RAM)")
        cli(cdc_port, f"rm {SCRIPT_PATH}\nexit\n")
        return 1
    print(f"  rand() advances: {','.join(vals)}")

    # Confirm the device is still responsive after the TLS-heavy path.
    try:
        alive = cli(cdc_port, "version\n", timeout=15)
    except subprocess.TimeoutExpired:
        print("FAIL: device unresponsive after rand()")
        return 1
    if "fantasi" not in alive:
        print("FAIL: device did not respond to version after rand()")
        return 1

    cli(cdc_port, f"rm {SCRIPT_PATH}\nexit\n")
    print("  device alive after rand(); TLS thread pointer is backed by RAM")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
