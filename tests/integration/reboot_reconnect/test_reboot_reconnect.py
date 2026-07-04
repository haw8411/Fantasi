#!/usr/bin/env python3
"""Verify a device reboot is reflected in the host CLI's connection status.

Drives the interactive CLI over a pty (pexpect) and checks that after a
``reboot`` the prompt turns red (disconnected) and then the CLI automatically
reconnects. The disconnect/reconnect detection lives in readline's idle event
hook, which only installs on a real TTY (piped stdin disables it) - so this
can't be exercised with the plain subprocess-pipe pattern the other CLI tests
use; it needs a pty.

Covers both transport shapes:
  - Composite (FZ/CU): the CLI runs over CDC serial with a persistent FAT mount.
    An ``ls`` is issued first to force the mount (msc_active=true), the state the
    composite disconnect path must still detect, then reboot should turn the
    prompt red and reconnect over serial.
  - Switch-mode (PM3): the CLI auto-upgrades to the WebUSB vendor pipe. After a
    reboot the device re-enumerates as CDC, so reconnect reopens serial and
    switches it back to WebUSB (``reconnected over USB``).
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")

RED = "\x1b[31m"   # C_RED - only the host's disconnected prompt emits this


def step(msg):
    print(f"  [{msg}]")


def fail(child, msg):
    print(f"  FAIL: {msg}")
    tail = (child.before or "")[-600:] if child else ""
    if tail:
        print("  --- last CLI output ---")
        print(repr(tail))
    return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", required=True)
    args = ap.parse_args()
    plat = PLATFORMS[args.platform]
    composite = plat["msc_mode"] == "composite"

    if not os.access(CLI_BIN, os.X_OK):
        print(f"SKIP: CLI binary not found at {CLI_BIN}")
        return 77

    import pexpect

    usb = find_usb_device(USB_VID, USB_PID)
    if not usb:
        print("  FAIL: no Fantasi device found")
        return 1
    cdc = find_cdc_port(usb)
    if not cdc or not cdc.startswith("/dev/tty"):
        print(f"  FAIL: no CDC serial port (got {cdc!r})")
        return 1
    step(f"device on {cdc}")

    child = pexpect.spawn(CLI_BIN, [cdc], encoding="utf-8",
                          codec_errors="replace", timeout=25)
    try:
        # Connected: the live (uncolored) prompt appears. (Switch-mode auto-
        # upgrades to WebUSB first, which takes a moment.)
        child.expect_exact("fantasi> ")
        step("connected (live prompt)")

        if composite:
            # Force the FAT mount so msc_active is set before the reboot - the
            # mounted state the composite disconnect path must still detect.
            child.sendline("ls")
            child.expect_exact("fantasi> ", timeout=30)
            step("storage mounted (ls ran)")

        # Reboot; the idle hook should color the prompt red once the link drops.
        child.sendline("reboot")
        child.expect(re.escape(RED) + "fantasi", timeout=20)
        step("disconnect reflected: prompt turned red")

        # ...and reconnect once the device re-enumerates (serial for composite,
        # a CDC->WebUSB re-upgrade for switch-mode).
        child.expect_exact("reconnected", timeout=60)
        step("auto-reconnected")

        # Live again: a command round-trips.
        child.expect_exact("fantasi> ")
        child.sendline("whoami")
        child.expect_exact("fantasi> ", timeout=15)
        step("CLI live after reconnect")

        child.sendline("exit")
        child.expect(pexpect.EOF, timeout=10)
        print("  PASS")
        return 0
    except pexpect.TIMEOUT:
        return fail(child, "timed out waiting for expected CLI state")
    except pexpect.EOF:
        return fail(child, "CLI exited unexpectedly")
    finally:
        if child.isalive():
            child.close(force=True)


if __name__ == "__main__":
    sys.exit(main())
