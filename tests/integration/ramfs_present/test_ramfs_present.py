#!/usr/bin/env python3
"""Validate that the synthetic /ramfs mount appears in the root listing over BOTH
storage paths:

  - MSC: host CLI `ls /` over USB - mounts the synthetic FAT, reads the root dir
    (firmware: build_model -> vfs_list).
  - BLE: host CLI `ls` over the protobuf transport - proto dir_list -> vfs_list.

/ramfs is RAM-backed, not a real LittleFS entry, so it only shows if the firmware
injects it. MSC injects it via build_model; the proto dir_list must go through
the same vfs_list (not read LittleFS directly), or `ls` over BLE would hide
/ramfs. This test guards that both paths stay in sync.

Result codes:
  - MSC listing is checked on every platform and FAILs hard if /ramfs is missing.
  - BLE is only checked on chameleon/flipper. If BLE is environmentally
    unavailable (no bluetoothctl/pexpect, or pairing can't complete) the BLE half
    SKIPs; the Proxmark3 has no BLE, so MSC alone is conclusive there.
"""

import argparse
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))                 # tests/integration (lib)
sys.path.insert(0, os.path.join(HERE, "..", "ble_speed"))    # reuse BLE pairing helpers

from lib.device import USB_VID, USB_PID, find_usb_device, find_cdc_port
import test_ble_speed as bsp     # device_ble_name / find_ble_addr / pair / PasskeyReader

REPO_ROOT = os.path.abspath(os.path.join(HERE, "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
BLE_PLATFORMS = ("chameleon", "flipper")


def cli(script, port=None, ble_addr=None, timeout=90):
    """Run a CLI session over USB (port) or BLE (ble_addr); ANSI-stripped output."""
    args = [CLI_BIN]
    args += [f"--ble-addr={ble_addr}", "--ble"] if ble_addr else [port]
    r = subprocess.run(args, input=script, capture_output=True, text=True, timeout=timeout)
    return re.sub(r'\033\[[0-9;]*m', '', r.stdout + r.stderr)


def ramfs_listed(out):
    """True if a 'ramfs' directory entry is present. MSC prints '  ramfs/';
    the proto listing prints '  ramfs                 <dir>'."""
    return (re.search(r'(?m)^\s*ramfs/\s*$', out) is not None or
            re.search(r'(?m)^\s*ramfs\s+<dir>\s*$', out) is not None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", required=True)
    args = ap.parse_args()

    if not os.path.isfile(CLI_BIN):
        print(f"SKIP: CLI binary not found at {CLI_BIN}")
        return 77

    usb = find_usb_device(USB_VID, USB_PID)
    if not usb:
        print("FAIL: Fantasi device not found")
        return 1
    cdc = find_cdc_port(usb)
    if not cdc:
        print("FAIL: CDC port not found")
        return 1
    print(f"  CDC: {cdc}")

    # ---- MSC path (every platform) ----
    print("  [MSC] ls /")
    out = cli("ls /\nexit\n", port=cdc)
    if not ramfs_listed(out):
        print("FAIL: /ramfs missing from the MSC listing")
        print(f"  output: {out[:400]}")
        return 1
    print("  [MSC] /ramfs present")

    # ---- BLE path (chameleon/flipper only) ----
    if args.platform not in BLE_PLATFORMS:
        print("  [BLE] not applicable on this platform (no BLE) - MSC is conclusive")
        print("PASS")
        return 0

    if not bsp.have("bluetoothctl"):
        print("SKIP: bluetoothctl not available (MSC half passed)")
        return 77
    import importlib.util
    for mod in ("serial", "pexpect"):       # PasskeyReader / pair() import these lazily
        if importlib.util.find_spec(mod) is None:
            print(f"SKIP: python BLE dep '{mod}' missing (MSC half passed)")
            return 77

    # Clear stale BlueZ state so the scan/pair is reliable (mirrors ble_speed).
    subprocess.run(["bluetoothctl", "power", "off"], capture_output=True, timeout=10)
    time.sleep(2)
    subprocess.run(["bluetoothctl", "power", "on"], capture_output=True, timeout=10)
    time.sleep(2)

    print("  [BLE] locating + pairing...")
    name = bsp.device_ble_name(cdc)            # pin to THIS board (stable per-chip name)
    addr = bsp.find_ble_addr(name)
    if not addr:
        print("SKIP: no 'Fantasi' BLE advertiser found (MSC half passed)")
        return 77

    reader = bsp.PasskeyReader(cdc)
    reader.start()
    try:
        paired = bsp.pair(addr, reader)
    finally:
        reader.stop()
    if paired is None:
        print("SKIP: pexpect unavailable for pairing (MSC half passed)")
        return 77
    if not paired:
        print("SKIP: could not pair - environmental (MSC half passed)")
        return 77

    print("  [BLE] ls")
    try:
        out = cli("ls\nexit\n", ble_addr=addr, timeout=60)
    finally:
        bsp.ble_disconnect(addr)

    if not ramfs_listed(out):
        print("FAIL: /ramfs missing from the BLE/proto listing")
        print(f"  output: {out[:400]}")
        return 1
    print("  [BLE] /ramfs present")

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
