#!/usr/bin/env python3
"""Fantasi integration-test runner.

Discovers test_*.py scripts under tests/integration/ and runs each one,
passing --platform through. These need a connected device; the hardware-free
unit tests run separately via run_unit.py (make test-unit). Exit codes:
0 = pass, 77 = skip, anything else = fail.
"""

import argparse
import glob
import os
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description="Fantasi test runner")
    parser.add_argument("--platform", required=True)
    args = parser.parse_args()

    test_dir = os.path.dirname(os.path.abspath(__file__))
    # Integration tests live under tests/integration/ (need a device); the
    # hardware-free tests/unit/ run separately via run_unit.py.
    tests = sorted(glob.glob(os.path.join(test_dir, "integration", "*", "test_*.py")))

    if not tests:
        print("No tests found.")
        return 0

    # A switch-mode device (PM3) is switched to WebUSB once (the first test's CLI
    # auto-upgrades it) and the whole suite then runs over WebUSB - find_cdc_port
    # hands the CLI a WebUSB sentinel once the CDC port is gone, so each per-test
    # CLI invocation still reaches the device. No need to reset to CDC between
    # tests (just avoids pointless mode switches); ensure_cdc runs once at the end
    # to leave the device back on serial.
    sys.path.insert(0, os.path.join(test_dir, "integration"))
    try:
        from lib.device import ensure_cdc
    except Exception:
        ensure_cdc = None

    results = []
    for test in tests:
        name = os.path.basename(os.path.dirname(test))
        print(f"\n{'=' * 60}")
        print(f"  {name}")
        print(f"{'=' * 60}\n")

        r = subprocess.run([sys.executable, test, "--platform", args.platform])

        if r.returncode == 0:
            results.append((name, "PASS"))
        elif r.returncode == 77:
            results.append((name, "SKIP"))
        else:
            results.append((name, "FAIL"))

    # Leave the device on serial (CDC) so it's in a predictable state afterward.
    if ensure_cdc is not None:
        ensure_cdc()

    print(f"\n{'=' * 60}")
    for name, status in results:
        print(f"  {status}  {name}")
    print(f"{'=' * 60}")

    if any(s == "FAIL" for _, s in results):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
