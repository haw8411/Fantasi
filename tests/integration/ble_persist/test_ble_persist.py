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
    USB_VID, USB_PID,
    find_usb_device, find_cdc_port,
    wait_for_usb,
    send_serial_cmd,
)


def step(msg):
    print(f"  [{msg}]")


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


def wait_reboot(timeout=20):
    # 1. reboot started - device drops off USB
    deadline = time.time() + 8
    while time.time() < deadline:
        if not find_usb_device(USB_VID, USB_PID):
            break
        time.sleep(0.2)
    # 2. device re-enumerates
    usb_dev = wait_for_usb(USB_VID, USB_PID, timeout=timeout)
    if not usb_dev:
        return None
    # 3. CLI is ready
    deadline = time.time() + 10
    while time.time() < deadline:
        port = find_cdc_port(usb_dev)
        if port and _cli_ready(port):
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
    if not cdc_port:
        print("FAIL: CDC port not found")
        return 1
    print(f"  CDC: {cdc_port}")

    # ---- Test 1: BLE off persists ----
    step("Setting BLE off")
    r = send_serial_cmd(cdc_port, "ble off", timeout=5)
    if "failed" in r:
        print(f"FAIL: ble off failed: {r}")
        return 1

    step("Rebooting")
    send_serial_cmd(cdc_port, "reboot", timeout=1)
    cdc_port = wait_reboot()
    if not cdc_port:
        print("FAIL: device did not reboot")
        return 1

    step("Checking BLE state after reboot (expect off)")
    r = send_serial_cmd(cdc_port, "radio", timeout=5)
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
    r = send_serial_cmd(cdc_port, "ble on", timeout=8)
    if "failed" in r:
        print(f"FAIL: ble on failed: {r}")
        return 1

    step("Rebooting")
    send_serial_cmd(cdc_port, "reboot", timeout=1)
    cdc_port = wait_reboot()
    if not cdc_port:
        print("FAIL: device did not reboot")
        return 1

    step("Checking BLE state after reboot (expect on)")
    r = send_serial_cmd(cdc_port, "radio", timeout=5)
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
