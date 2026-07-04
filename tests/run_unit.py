#!/usr/bin/env python3
"""Fantasi unit-test runner.

Discovers and runs the hardware-free tests under tests/unit/. Unlike run.py
(integration tests, which need a connected device and a --platform), these run
anywhere - they're what CI executes on every push. Exit codes per test:
0 = pass, 77 = skip, anything else = fail. The runner exits non-zero if any
test failed.
"""

import glob
import os
import subprocess
import sys


def main():
    test_dir = os.path.dirname(os.path.abspath(__file__))
    tests = sorted(glob.glob(os.path.join(test_dir, "unit", "test_*.py")))

    if not tests:
        print("No unit tests found.")
        return 0

    results = []
    for test in tests:
        name = os.path.basename(test)[len("test_"):-len(".py")]
        print(f"\n{'=' * 60}\n  {name}\n{'=' * 60}\n")
        r = subprocess.run([sys.executable, test])
        if r.returncode == 0:
            results.append((name, "PASS"))
        elif r.returncode == 77:
            results.append((name, "SKIP"))
        else:
            results.append((name, "FAIL"))

    print(f"\n{'=' * 60}")
    for name, status in results:
        print(f"  {status}  {name}")
    print(f"{'=' * 60}")

    return 1 if any(s == "FAIL" for _, s in results) else 0


if __name__ == "__main__":
    sys.exit(main())
