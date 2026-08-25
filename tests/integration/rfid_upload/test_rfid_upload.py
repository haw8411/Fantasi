#!/usr/bin/env python3
"""Exercise the RFID command's private protobuf response parser.

The RFID host command uploads its driver before entering its own event loop and
does not use main.c's shared response parser. This verifies mailbox handoff and
upload progress independently of that parser.
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import PLATFORMS, USB_VID, USB_PID, find_usb_device

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")
MOCK_MFC_SOURCE = os.path.join(os.path.dirname(__file__), "mock_mfc_read.c")
TRANSPORT_ARGS = ["--usb"]


def run(command, app_dir, timeout=20):
    env = os.environ.copy()
    env["FANTASI_APP_DIR"] = app_dir
    started = time.monotonic()
    result = subprocess.run(
        [CLI_BIN, *TRANSPORT_ARGS, "-c", command],
        env=env, capture_output=True, text=True, timeout=timeout,
    )
    return result, time.monotonic() - started


def failed(result):
    text = (result.stdout + result.stderr).lower()
    return result.returncode != 0 or "driver upload failed" in text or "launch failed" in text


def heap_free(text):
    match = re.search(r"heap:\s*(\d+)/", text)
    return int(match.group(1)) if match else None


def build_mock_app_dir(root, driver, arch, flavor):
    """Build a deterministic mfc_read module beside the real RFID driver."""
    outdir = os.path.join(root, flavor)
    os.mkdir(outdir)
    os.symlink(os.path.realpath(driver), os.path.join(outdir, f"rfid.{arch}.elf"))
    obj = os.path.join(outdir, "mfc_read.o")
    elf = os.path.join(outdir, f"mfc_read.{arch}.elf")
    arch_flags = (["-O2", "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=hard",
                   "-mfpu=fpv4-sp-d16"] if arch == "cm4" else
                  ["-Os", "-mcpu=arm7tdmi", "-mthumb-interwork"])
    define = {
        "valid": None,
        "malformed": "MOCK_MFC_MALFORMED",
        "blocked": "MOCK_MFC_BLOCK",
    }[flavor]
    compile_cmd = [
        "arm-none-eabi-gcc", *arch_flags,
        "-ffreestanding", "-fno-common", "-mword-relocations", "-mlong-calls",
        "-ffunction-sections", "-fdata-sections", "-nostdlib", "-Wall",
        "-I", os.path.join(REPO_ROOT, "apps"),
    ]
    if define:
        compile_cmd.append(f"-D{define}")
    compile_cmd += ["-c", MOCK_MFC_SOURCE, "-o", obj]
    subprocess.run(compile_cmd, check=True, capture_output=True, text=True)
    subprocess.run([
        "arm-none-eabi-ld", "-r", "-T", os.path.join(REPO_ROOT, "apps/app.ld"),
        obj, "-o", elf,
    ], check=True, capture_output=True, text=True)
    os.symlink(os.path.basename(elf), os.path.join(outdir, f"mfc_block.{arch}.elf"))
    return outdir


def main():
    global TRANSPORT_ARGS
    parser = argparse.ArgumentParser()
    parser.add_argument("--platform", required=True)
    parser.add_argument("--ble-addr", help="run the same suite through this BLE device")
    args = parser.parse_args()
    if args.platform not in PLATFORMS:
        print(f"FAIL: unknown platform {args.platform}")
        return 1
    if not os.path.isfile(CLI_BIN):
        print(f"SKIP: CLI binary not found at {CLI_BIN}")
        return 77
    if args.ble_addr:
        if args.platform in ("proxmark3", "proxmark5"):
            print(f"SKIP: {args.platform} has no BLE CLI transport")
            return 77
        TRANSPORT_ARGS = [f"--ble-addr={args.ble_addr}"]

    app_dir = os.environ.get("FANTASI_APP_DIR", "")
    arch = PLATFORMS[args.platform]["app_arch"]
    driver = os.path.join(app_dir, f"rfid.{arch}.elf") if app_dir else ""
    if not driver or not os.path.isfile(driver):
        print("SKIP: set FANTASI_APP_DIR to an app build containing "
              f"rfid.{arch}.elf")
        return 77
    if not args.ble_addr and not find_usb_device(USB_VID, USB_PID):
        print("FAIL: Fantasi USB device not found")
        return 1

    # One `version` frame is followed by many short FileWrite ACKs. Each frame
    # must begin at offset zero and release the preceding mailbox.
    result, elapsed = run("rfid help", app_dir)
    if failed(result) or "commands:" not in result.stdout:
        print("FAIL: RFID driver did not upload and launch")
        print((result.stdout + result.stderr)[:1200])
        return 1
    if elapsed > 8.0:
        print(f"FAIL: RFID driver upload was delayed ({elapsed:.2f}s)")
        return 1
    print(f"  RFID driver uploaded, launched, and answered in {elapsed:.2f}s")

    # Keep one device session for several launches. Best-effort cleanup replies
    # from the prior app must not be mistaken for the next version response.
    result, elapsed = run("rfid help;rfid help;rfid help", app_dir, timeout=30)
    if failed(result) or result.stdout.count("commands:") != 3:
        print("FAIL: repeated RFID launches desynchronized one WebUSB session")
        print((result.stdout + result.stderr)[:1200])
        return 1
    print(f"  three back-to-back launches in one session: {elapsed:.2f}s")

    # Invalid input must fail before touching RF or requesting a module. Cover
    # malformed T5577 data and commands that exceed the app's input buffer.
    parser_cases = (
        ("rfid write t5577 -b 4 -d XYZ", "exactly 8 hex digits"),
        ("rfid read mfc -d DEADBEEF", "-d is only valid with write"),
        ("rfid write t5577 -b 4 -d 12345678 -s", "-s is only valid with read"),
        ("rfid read mfc -b 64", "block must be 0-63"),
        ("rfid read mfc -k FFFFFFFF", "key must be exactly 12 hex digits"),
        ("rfid sniff", "usage: sniff <protocol>"),
        ("rfid raw mfc -z 3000", "unknown option '-z'"),
        ("rfid trace junk", "usage: trace [clear]"),
        ("rfid raw mfc " + "AA" * 40, "command too long"),
    )
    for command, expected in parser_cases:
        result, _ = run(command, app_dir)
        text = result.stdout + result.stderr
        if failed(result) or expected not in text:
            print(f"FAIL: RFID parser did not reject {command!r} safely")
            print(text[:1200])
            return 1
    print("  invalid flags/data and overlong commands rejected before RF: OK")

    listed, _ = run("rfid list nfca", app_dir)
    listed_text = re.sub(r"\x1b\[[0-9;]*m", "", listed.stdout + listed.stderr)
    mfc_line = next((line for line in listed_text.splitlines()
                     if re.match(r"\s*mfc\s+MIFARE Classic", line)), "")
    if failed(listed) or "collect" not in mfc_line or "emulate" not in mfc_line:
        print("FAIL: MFC collect/emulate are missing from `rfid list`")
        print(listed_text[:1600])
        return 1
    print("  list reports MFC read/sniff/raw/emulate/collect per operation: OK")

    # Exercise the entire dynamic-module and host-rendering path with
    # deterministic records. This proves that 32 valid keys + 64 block statuses
    # render, malformed-but-present records do not count, and SIGINT can abort a
    # genuinely blocked read without depending on a physical card's placement.
    try:
        with tempfile.TemporaryDirectory(prefix="fantasi-mfc-contract-") as td:
            valid_dir = build_mock_app_dir(td, driver, arch, "valid")
            malformed_dir = build_mock_app_dir(td, driver, arch, "malformed")
            blocked_dir = build_mock_app_dir(td, driver, arch, "blocked")

            # Availability is per operation, not "some module for this
            # protocol exists". This fixture deliberately contains read but
            # not collect/emulate, independent of what future real builds add.
            partial_list, _ = run("rfid list nfca", valid_dir)
            partial_text = re.sub(
                r"\x1b\[[0-9;]*m", "", partial_list.stdout + partial_list.stderr)
            partial_mfc = next((line for line in partial_text.splitlines()
                                if re.match(r"\s*mfc\s+MIFARE Classic", line)), "")
            if (failed(partial_list) or "read*" in partial_mfc or
                    "collect*" not in partial_mfc or "emulate*" not in partial_mfc):
                print("FAIL: RFID list availability was not tracked per operation")
                print(partial_text[:1600])
                return 1

            valid, _ = run("rfid read mfc", valid_dir)
            valid_text = valid.stdout + valid.stderr
            if (failed(valid) or "64/64 blocks" not in valid_text or
                    valid_text.count("SECTOR ") != 16):
                print("FAIL: complete 32-key/64-block MFC records did not render")
                print(valid_text[:1600])
                return 1

            one_block, _ = run(
                "rfid read mfc -b 17 -k A0A1A2A3A4A5", valid_dir)
            one_block_text = re.sub(
                r"\x1b\[[0-9;]*m", "", one_block.stdout + one_block.stderr)
            if (failed(one_block) or "read complete" not in one_block_text or
                    "block 17" not in one_block_text or
                    "key A A0A1A2A3A4A5" not in one_block_text or
                    "64/64 blocks" in one_block_text):
                print("FAIL: MFC -b/-k did not produce one authenticated block")
                print(one_block_text[:1600])
                return 1

            preferred, _ = run(
                "rfid read mfc -k A0A1A2A3A4A5", valid_dir)
            preferred_text = preferred.stdout + preferred.stderr
            if (failed(preferred) or "64/64 blocks" not in preferred_text or
                    preferred_text.count("SECTOR ") != 16 or
                    "key A A0A1A2A3A4A5" not in preferred_text):
                print("FAIL: a preferred MFC key changed the full-read contract")
                print(preferred_text[:1600])
                return 1

            malformed, _ = run("rfid read mfc", malformed_dir)
            malformed_text = malformed.stdout + malformed.stderr
            expected = "incomplete MIFARE Classic result (31/32 keys, 63/64 block statuses)"
            if failed(malformed) or expected not in malformed_text or "64/64 blocks" in malformed_text:
                print("FAIL: malformed MFC key/block records counted as complete")
                print(malformed_text[:1600])
                return 1

            baseline_run = subprocess.run(
                [CLI_BIN, *TRANSPORT_ARGS, "-c", "free"],
                capture_output=True, text=True, timeout=30)
            baseline = heap_free(baseline_run.stdout)
            env = os.environ.copy()
            env["FANTASI_APP_DIR"] = blocked_dir
            import pexpect
            blocked = pexpect.spawn(
                CLI_BIN, [*TRANSPORT_ARGS, "-c",
                          "rfid read mfc -b 17 -k A0A1A2A3A4A5"], env=env,
                encoding="utf-8", codec_errors="replace", timeout=30)
            try:
                # BLE service discovery + driver upload can take several
                # seconds. Signal only after app_capture has actually begun,
                # so this tests command cancellation rather than startup.
                blocked.expect("reading:")
                blocked.kill(signal.SIGINT)
                blocked.expect_exact("read: cancelled", timeout=10)
                blocked.expect(pexpect.EOF, timeout=10)
                blocked.close()
                if blocked.exitstatus not in (None, 0):
                    raise RuntimeError(f"RFID host exited {blocked.exitstatus}")
            except (pexpect.TIMEOUT, pexpect.EOF, RuntimeError) as exc:
                transcript = blocked.before or ""
                if blocked.isalive():
                    blocked.close(force=True)
                print(f"FAIL: blocked MFC read did not cancel cleanly: {exc}")
                print(transcript[:1600])
                return 1
            time.sleep(0.25)
            health = subprocess.run(
                [CLI_BIN, *TRANSPORT_ARGS, "-c", "free;ps"],
                capture_output=True, text=True, timeout=30)
            after = heap_free(health.stdout)
            if (baseline is None or after != baseline or
                    re.search(r"^\s*\d+\s+(?:app|apppump)\s", health.stdout, re.MULTILINE)):
                print("FAIL: cancelled MFC read left an app task or heap allocation")
                print(health.stdout[:1600])
                return 1
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        print(f"FAIL: could not exercise the synthetic MFC contract: {exc}")
        return 1
    print("  MFC contract: 32 keys + 64 statuses required; blocked read cancels cleanly")

    # No broker or shared host owner: another process must independently OPEN
    # and complete short commands while the first repeatedly uploads/apps/exits.
    env = os.environ.copy()
    env["FANTASI_APP_DIR"] = app_dir
    rfid = subprocess.Popen(
        [CLI_BIN, *TRANSPORT_ARGS, "-c", ";".join(["rfid help"] * 6)],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    time.sleep(0.1)
    peers = [subprocess.Popen(
        [CLI_BIN, *TRANSPORT_ARGS, "-c", "whoami"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    ) for _ in range(4)]
    try:
        for peer in peers:
            out, err = peer.communicate(timeout=15)
            if peer.returncode or not out.strip() or "error" in err.lower():
                print("FAIL: independent command failed during RFID uploads")
                print((out + err)[:600])
                return 1
        out, err = rfid.communicate(timeout=30)
        rfid_text = (out + err).lower()
        if (rfid.returncode or "driver upload failed" in rfid_text or
                "launch failed" in rfid_text or out.count("commands:") != 6):
            print("FAIL: RFID upload failed under independent WebUSB traffic")
            print((out + err)[:1200])
            return 1
    except subprocess.TimeoutExpired:
        print("FAIL: concurrent RFID/WebUSB test timed out")
        return 1
    finally:
        for proc in peers + [rfid]:
            if proc.poll() is None:
                proc.kill()
                proc.communicate()

    print("  RFID uploads + four independent whoami processes: OK")
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
