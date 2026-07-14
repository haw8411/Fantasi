#!/usr/bin/env python3
"""Unit test: the committed Berry const tables match berry_conf.h + the source.

Regenerates third_party/berry/generate/ with Berry's `coc` codegen (same
invocation as `make berry`) into a temp dir and diffs against the committed
files. Fails if they have drifted - fix with `make berry`. The coc tool is
vendored (third_party/berry/tools/coc/), so this is a strict, deterministic
comparison. Hardware-free - safe for CI.
"""
import argparse
import filecmp
import os
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
BERRY = os.path.join(REPO_ROOT, "third_party/berry")
COMMITTED = os.path.join(BERRY, "generate")


def skip(msg):
    print(f"SKIP: {msg}")
    sys.exit(77)


def fail(msg):
    print(f"FAIL: {msg}")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", default=None)   # ignored; accepted for the runner
    ap.parse_args()

    coc = os.path.join(BERRY, "tools/coc/coc")
    if not os.path.isfile(coc):
        skip("coc tool not vendored")
    if not os.path.isdir(COMMITTED):
        fail("third_party/berry/generate/ missing - run `make berry`")

    with tempfile.TemporaryDirectory() as tmp:
        r = subprocess.run(
            [sys.executable, coc, "-o", tmp, os.path.join(BERRY, "src"),
             "-c", os.path.join(BERRY, "berry_conf.h")],
            capture_output=True, text=True)
        if r.returncode != 0:
            fail("coc failed:\n" + r.stdout + r.stderr)

        committed = set(os.listdir(COMMITTED))
        fresh = set(os.listdir(tmp))
        if committed != fresh:
            fail("generated file set drifted (run `make berry`):\n"
                 f"  only committed: {sorted(committed - fresh)}\n"
                 f"  only fresh:     {sorted(fresh - committed)}")

        drifted = [f for f in fresh
                   if not filecmp.cmp(os.path.join(tmp, f),
                                      os.path.join(COMMITTED, f), shallow=False)]
        if drifted:
            fail("committed Berry tables drifted from berry_conf.h+src "
                 f"(run `make berry`): {sorted(drifted)}")

    print("PASS: Berry const tables in sync")


if __name__ == "__main__":
    main()
