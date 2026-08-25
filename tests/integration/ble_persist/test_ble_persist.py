#!/usr/bin/env python3
"""Verify that BLE on/off state persists across reboot.

Sets BLE off, reboots, confirms BLE is off via ``radio`` and
``settings``.  Then sets BLE on, reboots, confirms it came back on.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    USB_VID, USB_PID, CLI_WEBUSB_SENTINEL,
    find_usb_device, find_cdc_port,
    wait_for_usb,
)


def step(msg):
    print(f"  [{msg}]")


def send_serial_cmd(port, cmd, timeout=2):
    """Send one CDC command and wait for its complete prompt-delimited reply."""
    import serial
    raw = bytearray()
    try:
        with serial.Serial(port, 115200, timeout=0.05) as ser:
            # Synchronize on one complete pre-command prompt. A fixed sleep followed by reset_input_buffer()
            # can leave the tail of that prompt in flight; mistaking it for the post-command prompt was the
            # reason slow `ble on` and immediate post-reboot `settings` replies were read as only 0-2 bytes.
            ser.reset_input_buffer()
            ser.write(b"\r")
            sync = bytearray()
            sync_deadline = time.monotonic() + 1.5
            while time.monotonic() < sync_deadline and b"fantasi>" not in sync:
                sync.extend(ser.read(ser.in_waiting or 1))
            ser.reset_input_buffer()
            ser.write(f"{cmd}\r".encode())

            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    raw.extend(chunk)
                    if b"fantasi>" in raw:
                        break
    except (OSError, serial.SerialException):
        pass

    text = raw.decode(errors="replace")
    lines = [
        line for line in text.splitlines()
        if line and not line.startswith("fantasi>") and line != cmd
        and "CLI ready" not in line
    ]
    return "\n".join(lines)


def send_serial_reply(port, cmd, timeout=2, attempts=3):
    """Retry only a missing CDC reply; a reported device error is returned verbatim."""
    reply = ""
    for attempt in range(attempts):
        reply = send_serial_cmd(port, cmd, timeout=timeout)
        if reply:
            break
        if attempt + 1 < attempts:
            time.sleep(0.25)
    return reply


def _cli_ready(port):
    """True once the CLI prompt responds on `port`."""
    import serial
    try:
        with serial.Serial(port, 115200, timeout=1) as s:
            s.write(b"\r\n")
            time.sleep(0.2)
            out = s.read(s.in_waiting or 64).decode(errors="replace")
        return "fantasi>" in out
    except Exception:
        return False


def _usb_devnum(usb_dev):
    """Return the kernel enumeration number for one /sys USB device."""
    if not usb_dev:
        return None
    try:
        with open(os.path.join(usb_dev, "devnum"), encoding="ascii") as f:
            return f.read().strip()
    except OSError:
        return None


def wait_reboot(previous_devnum, timeout=20):
    # 1. Reboot started.  A fast reset can disappear and re-enumerate between
    # 200 ms polls, so accept either a visible absence or a changed kernel USB
    # enumeration number.  This still proves a reset occurred; mere continued
    # CLI responsiveness on the old device is not enough.
    deadline = time.time() + 8
    reenumerated = False
    while time.time() < deadline:
        usb_dev = find_usb_device(USB_VID, USB_PID)
        if not usb_dev or (
            previous_devnum is not None
            and _usb_devnum(usb_dev) != previous_devnum
        ):
            reenumerated = True
            break
        time.sleep(0.05)
    if not reenumerated:
        return None
    # 2. device re-enumerates
    usb_dev = wait_for_usb(USB_VID, USB_PID, timeout=timeout)
    if not usb_dev:
        return None
    # 3. CLI is ready
    deadline = time.time() + 10
    while time.time() < deadline:
        port = find_cdc_port(usb_dev)
        if port not in (None, CLI_WEBUSB_SENTINEL) and _cli_ready(port):
            return port
        time.sleep(0.3)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--platform", required=True)
    args = parser.parse_args()

    if args.platform in ("proxmark3", "proxmark5"):
        print(f"SKIP: {args.platform} has no BLE")
        return 77

    usb_dev = find_usb_device(USB_VID, USB_PID)
    if not usb_dev:
        print("FAIL: Fantasi device not found")
        return 1

    cdc_port = find_cdc_port(usb_dev)
    if cdc_port in (None, CLI_WEBUSB_SENTINEL):
        print(f"FAIL: real CDC port not found (got {cdc_port!r})")
        return 1
    print(f"  CDC: {cdc_port}")

    # ---- Test 1: BLE off persists ----
    step("Setting BLE off")
    r = send_serial_reply(cdc_port, "ble off", timeout=5)
    if "ble off" not in r.lower() or "failed" in r.lower():
        print(f"FAIL: ble off did not complete: {r!r}")
        return 1

    step("Rebooting")
    previous_devnum = _usb_devnum(usb_dev)
    send_serial_cmd(cdc_port, "reboot", timeout=1)
    cdc_port = wait_reboot(previous_devnum)
    if not cdc_port:
        print("FAIL: device did not reboot")
        return 1

    step("Checking BLE state after reboot (expect off)")
    r = send_serial_reply(cdc_port, "radio", timeout=5)
    if "BLE:" in r and "off" in r.split("BLE:")[-1].split("\n")[0].lower():
        print("  radio: BLE off")
    else:
        print(f"FAIL: radio does not show BLE off: {r}")
        return 1

    r = ""
    for _ in range(4):   # the CLI can be slow to answer right after a reboot
        r = send_serial_cmd(cdc_port, "settings", timeout=5)
        if "ble=0" in r:
            break
        time.sleep(0.5)
    if "ble=0" in r:
        print("  settings: ble=0")
    else:
        print(f"FAIL: settings does not show ble=0: {r}")
        return 1

    # ---- Test 2: BLE on persists ----
    step("Setting BLE on")
    r = send_serial_reply(cdc_port, "ble on", timeout=8)
    if "ble on" not in r.lower() or "failed" in r.lower():
        print(f"FAIL: ble on did not complete: {r!r}")
        return 1

    step("Rebooting")
    usb_dev = find_usb_device(USB_VID, USB_PID)
    previous_devnum = _usb_devnum(usb_dev)
    send_serial_cmd(cdc_port, "reboot", timeout=1)
    cdc_port = wait_reboot(previous_devnum)
    if not cdc_port:
        print("FAIL: device did not reboot")
        return 1

    step("Checking BLE state after reboot (expect on)")
    r = send_serial_reply(cdc_port, "radio", timeout=5)
    if "BLE:" in r and "on" in r.split("BLE:")[-1].split("\n")[0].lower():
        print("  radio: BLE on")
    else:
        print(f"FAIL: radio does not show BLE on: {r}")
        return 1

    r = ""
    for _ in range(4):   # the CLI can be slow to answer right after a reboot
        r = send_serial_cmd(cdc_port, "settings", timeout=5)
        if "ble=1" in r:
            break
        time.sleep(0.5)
    if "ble=1" in r:
        print("  settings: ble=1")
    else:
        print(f"FAIL: settings does not show ble=1: {r}")
        return 1

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
