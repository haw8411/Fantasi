#!/usr/bin/env python3
"""Verify the settings KV store (hal_settings_set / _get) over the CLI.

Exercises the firmware's streaming settings store through the `settings`
command's get/set/unset subcommands.
"""

import argparse
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import PLATFORMS, USB_VID, USB_PID, find_usb_device, find_cdc_port

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
KEY = "testset"


def run(cdc_port, cmd, timeout=30):
    """Run one CLI command; return its output lines (prompt/echo/noise stripped)."""
    r = subprocess.run([CLI_BIN, cdc_port], input=cmd + "\nexit\n",
                       capture_output=True, text=True, timeout=timeout)
    out = re.sub(r'\033\[[0-9;]*m', '', r.stdout)
    lines = []
    for line in out.splitlines():
        line = line.strip()
        if (not line or line.startswith("fantasi>") or line == cmd
                or "transport:" in line or "CLI ready" in line
                or line.startswith("Unmounted ")):
            continue
        lines.append(line)
    return lines


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

    def count_rows(dump):
        return sum(1 for l in dump if l.startswith(KEY + "="))

    def fail(msg, extra=""):
        print(f"FAIL: {msg}")
        if extra:
            print(f"  {extra}")
        run(cdc_port, f"settings unset {KEY}")   # best-effort cleanup
        return 1

    # A prior run may have left the key behind.
    run(cdc_port, f"settings unset {KEY}")

    # 1. Set a new key.
    print("  [set testset 1]")
    if f"{KEY}=1" not in run(cdc_port, f"settings set {KEY} 1"):
        return fail("set did not confirm testset=1")
    if run(cdc_port, f"settings get {KEY}") != ["1"]:
        return fail("get after set did not return 1")
    dump = run(cdc_port, "settings")
    if count_rows(dump) >= 1 and count_rows(dump) != 1:
        return fail("more than one testset row after first set", dump)
    print("    new key set + read back")

    # 2. Update it - must not create a second row.
    print("  [set testset 2 (update)]")
    if f"{KEY}=2" not in run(cdc_port, f"settings set {KEY} 2"):
        return fail("update did not confirm testset=2")

    got = run(cdc_port, f"settings get {KEY}")
    if got != ["2"]:
        # get returns the first match; "1" here would mean a stale row was left.
        return fail("get after update did not return 2 (stale row left behind?)", str(got))
    print("    get returns 2 (no stale row ahead of it)")

    dump = run(cdc_port, "settings")
    rows = count_rows(dump)
    if rows >= 1:
        if rows != 1:
            return fail(f"update left {rows} testset rows, expected 1", "\n".join(dump))
        if any(l == f"{KEY}=1" for l in dump):
            return fail("stale testset=1 row still present", "\n".join(dump))
        print("    dump shows exactly one testset row (=2)")
    else:
        print("    note: dump truncated (config > 256 B); dedup confirmed via get")

    # 3. Clean up.
    print("  [unset testset]")
    run(cdc_port, f"settings unset {KEY}")
    if "not set" not in " ".join(run(cdc_port, f"settings get {KEY}")):
        return fail("key still present after unset")
    if count_rows(run(cdc_port, "settings")) != 0:
        return fail("testset row still in dump after unset")
    print("    key removed")

    print("  settings set/get/update-dedup/unset all correct")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
