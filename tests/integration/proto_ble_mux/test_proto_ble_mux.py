#!/usr/bin/env python3
"""Exercise independent BLE CLI processes on one BlueZ/device connection."""

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import PLATFORMS

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
ANSI = re.compile(r"\033\[[0-9;]*m")
UPTIME = re.compile(
    r"^(?:(\d+)d )?(?:(\d+)h )?(?:(\d+)m )?(\d+\.\d+)s$")
BLE_ARGS = ["--ble"]
FANTASI_SERVICE_UUIDS = (
    "8fe5b3d5-2e7f-4a98-2a48-7acc60fe0000",  # Flipper serial service
    "6e400001-b5a3-f393-e0a9-e50e24dcca9e",  # Nordic UART service
)


def cli(command, timeout=40):
    return subprocess.run([CLI_BIN, *BLE_ARGS, "-c", command],
                          capture_output=True, text=True, timeout=timeout)


def count_sessions(text):
    m = re.search(r"(\d+) protobuf sessions?", ANSI.sub("", text))
    return int(m.group(1)) if m else None


def running_ble_sessions(text):
    return len(re.findall(r"^\s*\d+\s+ble\s+\d+\s+running\s+",
                          ANSI.sub("", text), re.MULTILINE))


def free_bytes(text):
    m = re.search(r"heap:\s*(\d+)/", ANSI.sub("", text))
    return int(m.group(1)) if m else None


def command_output(text):
    return [line.strip() for line in ANSI.sub("", text).splitlines()
            if line.strip() and not line.startswith("ble:") and
            not line.startswith("transport:")]


def uptime_values(text):
    values = []
    for line in ANSI.sub("", text).replace("\r", "").splitlines():
        m = UPTIME.fullmatch(line.strip())
        if not m:
            continue
        days, hours, minutes, seconds = m.groups()
        values.append((int(days or 0) * 86400 + int(hours or 0) * 3600 +
                       int(minutes or 0) * 60 + float(seconds)))
    return values


def have_paired_fantasi():
    paired = subprocess.run(["bluetoothctl", "devices", "Paired"],
                            capture_output=True, text=True, timeout=10)
    if "Fantasi" in paired.stdout:
        return True
    # Match the same serial-service UUIDs as the production CLI so a valid
    # paired board is not skipped by its alias.
    for addr in re.findall(r"^Device\s+([0-9A-Fa-f:]{17})\s+", paired.stdout,
                           re.MULTILINE):
        info = subprocess.run(["bluetoothctl", "info", addr],
                              capture_output=True, text=True, timeout=10)
        lowered = info.stdout.lower()
        if any(uuid in lowered for uuid in FANTASI_SERVICE_UUIDS):
            return True
    return False


def skip(message):
    print(f"SKIP: {message}")
    return 77


def fail(message, detail=""):
    print(f"FAIL: {message}")
    if detail:
        print(detail[:1000])
    return 1


def main():
    global BLE_ARGS
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", required=True)
    ap.add_argument("--ble-addr",
                    help="pin every independent client to one BLE device")
    args = ap.parse_args()
    platform = args.platform
    if args.ble_addr:
        BLE_ARGS = [f"--ble-addr={args.ble_addr}"]
    if platform not in PLATFORMS:
        return fail(f"unknown platform {platform}")
    if platform in ("proxmark3", "proxmark5"):
        return skip(f"{platform} has no BLE CLI transport")
    if not os.path.isfile(CLI_BIN) or not shutil.which("bluetoothctl"):
        return skip("host CLI or bluetoothctl unavailable")

    # Pairing is interactive and is already covered by ble_speed. This test is
    # deterministic only with an existing bond; do not stop for a passkey.
    if not have_paired_fantasi():
        return skip("no paired Fantasi BLE device")

    try:
        preflight = cli("whoami", timeout=30)
    except subprocess.TimeoutExpired:
        return skip("paired BLE device is not reachable")
    if preflight.returncode:
        return skip("BLE preflight failed: " + preflight.stderr.strip())

    # A command such as ps writes one line at a time. Keep the transport from
    # turning those writes into one connection-interval/protobuf frame each.
    # Device timestamps exclude BlueZ connection and service-discovery time.
    paced = cli("uptime;ps;uptime", timeout=30)
    stamps = uptime_values(paced.stdout)
    if paced.returncode or len(stamps) != 2 or "STACKFREE" not in paced.stdout:
        return fail("could not measure complete BLE ps output",
                    paced.stdout + paced.stderr)
    ps_span = stamps[1] - stamps[0]
    if ps_span > 1.5:
        return fail(f"BLE ps output was paced line-by-line ({ps_span:.3f}s)",
                    paced.stdout)
    print(f"  500-byte BLE ps response: {ps_span * 1000:.0f} ms")

    base = cli("w")
    base_count = count_sessions(base.stdout)
    if base_count is None:
        return fail("w did not report a BLE session baseline", base.stdout + base.stderr)

    time.sleep(0.5)
    base_free = free_bytes(cli("free").stdout)
    if base_free is None:
        return fail("could not read the BLE idle-session heap baseline")

    # More than two independent subscribers is important on BlueZ: each
    # physical notification can be fanned out once per StartNotify owner. Hold
    # several idle processes open, verify their low-RAM device sessions, then
    # run a simultaneous command burst while every subscriber remains present.
    holders = []
    holder_count = 3
    try:
        for _ in range(holder_count):
            holders.append(subprocess.Popen(
                [CLI_BIN, *BLE_ARGS], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1))
        listed = None
        for _ in range(20):
            time.sleep(0.5)
            if any(p.poll() is not None for p in holders):
                return fail("an independent idle BLE CLI exited unexpectedly")
            listed = cli("w")
            if count_sessions(listed.stdout) is not None and \
                    count_sessions(listed.stdout) >= base_count + holder_count:
                break
        if listed is None or count_sessions(listed.stdout) is None or \
                count_sessions(listed.stdout) < base_count + holder_count:
            return fail("w did not show every independent idle BLE session",
                        "" if listed is None else listed.stdout)

        during_free = free_bytes(cli("free").stdout)
        if during_free is None:
            return fail("could not measure BLE idle-session heap")
        idle_cost = base_free - during_free
        if idle_cost < 0 or idle_cost > holder_count * 256:
            return fail(f"BLE idle session RAM cost is too high: {idle_cost} B "
                        f"for {holder_count}")
        print(f"  w: {count_sessions(listed.stdout)} sessions "
              f"({holder_count} independently held open)")
        print(f"  BLE idle-session heap delta: {idle_cost} B total")

        burst = [subprocess.Popen([CLI_BIN, *BLE_ARGS, "-c", "whoami"],
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, text=True)
                 for _ in range(3)]
        for p in burst:
            out, err = p.communicate(timeout=30)
            if p.returncode or not command_output(out) or "error:" in err.lower():
                return fail("concurrent BLE whoami process failed", out + err)
        print("  concurrent BLE whoami burst: OK")
    except subprocess.TimeoutExpired:
        return fail("idle-session BLE stress timed out")
    finally:
        for p in holders:
            if p.poll() is None and p.stdin:
                try:
                    p.stdin.write("exit\n")
                    p.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
        for p in holders:
            try:
                p.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                p.kill()
                p.communicate()

    # A long stream and a short request are distinct device sessions even though
    # BlueZ shares one physical BLE link and broadcasts the notifications.
    log = subprocess.Popen([CLI_BIN, *BLE_ARGS, "-c", "log"],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           text=True)
    try:
        # BlueZ may need a few seconds to establish another D-Bus notification
        # owner on an already-shared physical link. Wait until `w` proves that
        # the device session exists; process liveness alone could mean it is
        # still inside Connect/StartNotify.
        listed = None
        active = None
        for _ in range(10):
            time.sleep(0.5)
            if log.poll() is not None:
                out, err = log.communicate()
                return fail("BLE log process exited before cancellation", out + err)
            listed = cli("w")
            active = count_sessions(listed.stdout)
            # The `w` request itself is one running row; `log` must be the
            # second. Counting total sessions can be fooled briefly by the
            # preceding polling process while its CLOSE is being consumed.
            if running_ble_sessions(listed.stdout) >= 2:
                break
        if listed is None or running_ble_sessions(listed.stdout) < 2:
            return fail("BLE log session did not become visible", listed.stdout)
        short = cli("whoami", timeout=20)
        if short.returncode:
            return fail("BLE whoami blocked behind another process's log",
                        short.stdout + short.stderr)
        if log.poll() is not None:
            out, err = log.communicate()
            return fail("BLE log process exited during independent command", out + err)
        listed = cli("w")
        active = count_sessions(listed.stdout)
        if (active is None or active < base_count + 1 or
                running_ble_sessions(listed.stdout) < 2):
            return fail("w did not show both BLE processes", listed.stdout)
        log.send_signal(signal.SIGINT)
        log.communicate(timeout=20)
        if log.returncode:
            return fail("framed BLE cancellation did not stop log cleanly")
        print("  blocked log + independent whoami + cancellation: OK")
    except subprocess.TimeoutExpired:
        return fail("concurrent BLE command timed out")
    finally:
        if log.poll() is None:
            log.kill()
            log.communicate()

    # Each upload request spans ATT writes. Interleave two independent BlueZ
    # writers and verify that the firmware's per-SID assembler keeps them apart.
    with tempfile.TemporaryDirectory(prefix="fantasi-ble-mux-") as td:
        # Use LittleFS rather than RAMFS here.  This proves that concurrent
        # sessions retain distinct open file handles/caches while the chunks
        # interleave, in addition to exercising the per-SID ATT assemblers.
        payloads = [
            ("/blemux-a", "BLE-A-0123456789\n" * 55),
            ("/blemux-b", "BLE-B-abcdefghij\n" * 55),
        ]
        uploads = []
        for i, (remote, content) in enumerate(payloads):
            local = os.path.join(td, f"ble-{i}.txt")
            with open(local, "w", encoding="utf-8") as f:
                f.write(content)
            uploads.append(subprocess.Popen(
                [CLI_BIN, *BLE_ARGS, "-c", f"upload {local} {remote}"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True))
        for p in uploads:
            try:
                out, err = p.communicate(timeout=90)
            except subprocess.TimeoutExpired:
                p.kill(); p.communicate()
                return fail("concurrent BLE upload timed out")
            if p.returncode or "error:" in err.lower() or "failed" in err.lower():
                return fail("concurrent BLE upload failed", out + err)
        for remote, content in payloads:
            got = cli(f"cat {remote}", timeout=45)
            if content.strip() not in ANSI.sub("", got.stdout):
                return fail(f"BLE multiplexed data mismatch for {remote}",
                            got.stdout + got.stderr)
            cli(f"rm {remote}")
        print("  concurrent multi-fragment BLE uploads: OK")

    time.sleep(0.5)
    final = cli("w")
    if count_sessions(final.stdout) != base_count:
        return fail("closed BLE sessions remained in w", final.stdout)
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
