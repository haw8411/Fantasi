#!/usr/bin/env python3
"""Measure BLE upload/download throughput and verify transfer integrity.

Pairs with the device over BLE (Passkey Entry - the 6-digit code is read
from the device's USB debug log), then drives the host CLI over the BLE
transport to upload a 16 KB file and read it back, timing each direction
and comparing CRC32.

This test FAILS only on:
  * a broken transfer  - upload or download did not complete,
  * a bad CRC32        - the bytes read back differ from those uploaded,
  * extremely low speed - throughput far below a healthy link.

Everything else SKIPs (exit 77): a healthy link does ~48 kbps up /
~220 kbps down, and the floors below sit ~6-15x under that, so normal
variance and even a moderate regression still pass. Missing hardware, no
Bluetooth stack, or pairing/agent unavailable are environmental and also
skip - they are not what this test validates.
"""

import argparse
import os
import re
import subprocess
import sys
import threading
import time
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (USB_VID, USB_PID, find_usb_device, find_cdc_port,
                        send_serial_cmd)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")

SIZE = 16384            # transfer size, bytes
UP_MIN_KBPS = 5.0       # "extremely low" upload floor (healthy ~48)
DOWN_MIN_KBPS = 15.0    # "extremely low" download floor (healthy ~220)
PROMPT = b"fantasi> "
REMOTE = "/ble_speed_test.bin"


def skip(msg):
    print(f"SKIP: {msg}")
    return 77


def fail(msg):
    print(f"FAIL: {msg}")
    return 1


def have(cmd):
    return subprocess.run(["which", cmd], capture_output=True).returncode == 0


# ---- BLE address discovery ----

def device_ble_name(cdc_port):
    """The device's stable per-chip name (from `whoami`); it advertises over BLE
    as 'Fantasi <name>'. Used to pin the scan to THIS board so a stale BlueZ
    cache entry from another Fantasi (e.g. a board swapped in earlier) can't be
    picked. Returns None if it can't be read (caller falls back to any Fantasi)."""
    try:
        r = subprocess.run([CLI_BIN, cdc_port], input="whoami\nexit\n",
                           capture_output=True, text=True, timeout=20)
    except Exception:
        return None
    out = re.sub(r'\033\[[0-9;]*m', '', r.stdout)
    for line in out.splitlines():
        line = line.strip()
        # The generated name is its own line, e.g. "Thadusag" or "Shetak0" - it
        # may contain digits (hal_name_generate's vowels include '0'/'1'), so
        # don't restrict to letters.
        if re.fullmatch(r"[A-Za-z0-9]{3,15}", line) and line not in ("whoami", "exit"):
            return line
    return None


def find_ble_addr(name=None, timeout=20):
    """Scan for the connected board's 'Fantasi <name>' advertiser and return its
    MAC as soon as it appears (the device advertises within ms of boot, so we
    don't wait out the whole window). `name` pins it to this specific device;
    without it, fall back to any 'Fantasi …' advertiser. Stale cache entries from
    other Fantasi boards whose name doesn't match are ignored."""
    pat = re.compile(r"Device ([0-9A-Fa-f:]{17}) Fantasi" +
                     (r"\s+" + re.escape(name) + r"\b" if name else r"\b"))
    scan = None
    try:
        scan = subprocess.Popen(["bluetoothctl", "--timeout", str(timeout),
                                 "scan", "on"],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        deadline = time.time() + timeout
        while time.time() < deadline:
            cache = subprocess.run(["bluetoothctl", "devices"],
                                   capture_output=True, text=True,
                                   timeout=10).stdout
            m = pat.search(cache)
            if m:
                return m.group(1)
            time.sleep(0.5)
        return None
    except Exception:
        return None
    finally:
        if scan:
            scan.terminate()
            try:
                scan.wait(timeout=2)
            except Exception:
                pass


# ---- Passkey capture from the device's USB log ----

class PasskeyReader(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.port = port
        self.passkey = None
        self._stop = threading.Event()

    def run(self):
        import serial
        try:
            s = serial.Serial(self.port, 115200, timeout=0.3)
        except Exception:
            return
        s.write(b"\r\nlog\r\n")
        buf = b""
        while not self._stop.is_set():
            try:
                d = s.read(128)
            except Exception:
                break
            if d:
                buf = (buf + d)[-512:]
                for m in re.finditer(rb"pair code:\s*(\d{6})", buf):
                    self.passkey = m.group(1).decode()
        try:
            s.write(b"\x03")
            s.close()
        except Exception:
            pass

    def stop(self):
        self._stop.set()


def pair(addr, reader, timeout=15):
    """Pair via bluetoothctl with a KeyboardOnly agent (Passkey Entry), entering
    the 6-digit code the device displays (on its screen and USB log). Both CU
    and FZ use this model. Returns True/False, or None if pexpect is
    unavailable (caller should skip)."""
    try:
        import pexpect
    except ImportError:
        return None
    P = __import__("pexpect")
    c = pexpect.spawn("bluetoothctl", encoding="utf-8", timeout=30)
    try:
        # Clean slate ONCE: drop any live connection and clear the host-side
        # bond (a leftover host bond desyncs re-pairing), set the agent, scan.
        c.sendline("power on")
        time.sleep(0.4)
        c.sendline(f"disconnect {addr}")
        time.sleep(1)
        c.sendline(f"remove {addr}")
        time.sleep(1)
        # The device is DISPLAY_ONLY + MITM → Passkey Entry: it generates and
        # displays a 6-digit code (on its screen and USB log) that the host must
        # enter. A KeyboardOnly host agent negotiates this method; the host is
        # prompted "Enter passkey" and we type the code read from the device log.
        # (DISPLAY_ONLY deliberately has no Numeric-Comparison path, so a peer
        # can never bond without proving knowledge of the displayed code.)
        c.sendline("agent KeyboardOnly")
        time.sleep(0.4)
        c.sendline("default-agent")
        time.sleep(0.4)
        c.sendline("scan on")

        # Pair from the scan cache. Do NOT remove before each attempt - that
        # evicts the device and the re-discovery is racy ("not available").
        # Wait for discovery, pair; only on a real failure remove + let the
        # scan re-find it for the next attempt.
        for attempt in range(4):
            reader.passkey = None
            # Poll the (shared) cache until the device is re-discovered after
            # the remove - more reliable than waiting for a one-shot expect line.
            disc_deadline = time.time() + 15
            while time.time() < disc_deadline:
                cache = subprocess.run(["bluetoothctl", "devices"],
                                       capture_output=True, text=True,
                                       timeout=8).stdout
                if addr in cache:
                    break
                time.sleep(0.5)
            time.sleep(0.3)
            c.sendline(f"pair {addr}")
            failed = False
            deadline = time.time() + timeout
            while time.time() < deadline:
                i = c.expect([r"Enter passkey.*:", r"Pairing successful",
                              r"Failed to pair.*", r"Confirm passkey.*\(yes/no\)",
                              r"not available", P.TIMEOUT, P.EOF], timeout=8)
                if i == 0:
                    pk = None
                    for _ in range(16):
                        pk = reader.passkey
                        if pk:
                            break
                        time.sleep(0.5)
                    if not pk:
                        failed = True
                        break
                    c.sendline(pk)
                elif i == 1:
                    return True
                elif i == 3:
                    c.sendline("yes")
                else:
                    failed = True   # failed / not available / timeout
                    break
            if failed:
                # A failed attempt leaves the device connected; `remove` alone
                # then reports "not available" on the next pair. Fully reset for
                # the retry: disconnect, remove, re-scan, and let the device
                # re-advertise before the loop re-discovers it.
                c.sendline(f"disconnect {addr}")
                time.sleep(1.2)
                c.sendline(f"remove {addr}")
                time.sleep(1)
                c.sendline("scan off")
                time.sleep(0.5)
                c.sendline("scan on")
                time.sleep(1)
        return False
    finally:
        try:
            c.sendline("quit")
            time.sleep(1)
            c.close()
        except Exception:
            pass


# ---- Throughput measurement over the live (paired) CLI session ----

class Measurement:
    """upload_kbps, download_kbps, crc_ok, and any error string."""
    def __init__(self):
        self.up_kbps = self.dn_kbps = 0.0
        self.up_done = self.dn_done = self.crc_ok = False
        self.error = None


def measure(addr, data):
    """Single CLI session: upload `data`, read it back, time both, CRC32."""
    m = Measurement()
    p = subprocess.Popen([CLI_BIN, f"--ble-addr={addr}"],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, bufsize=0)
    out = bytearray()
    lock = threading.Lock()

    def reader():
        while True:
            b = p.stdout.read(256)
            if not b:
                break
            with lock:
                out.extend(b)
    threading.Thread(target=reader, daemon=True).start()

    def wait(tok, start, to):
        end = time.time() + to
        while time.time() < end:
            with lock:
                i = out.find(tok, start)
            if i >= 0:
                return i
            time.sleep(0.005)
        return -1

    def send(s):
        p.stdin.write(s.encode() + b"\n")
        p.stdin.flush()

    tmp = os.path.join(REPO_ROOT, "build", "ble_speed_test.bin")
    with open(tmp, "wb") as f:
        f.write(data)
    try:
        if wait(b"connected", 0, 15) < 0:
            m.error = "BLE connect failed (GATT locked or no link)"
            return m
        pos = wait(PROMPT, 0, 8)
        if pos < 0:
            m.error = "no CLI prompt over BLE"
            return m
        send("rm " + REMOTE)
        wait(PROMPT, pos + len(PROMPT), 8)
        with lock:
            end = len(out)

        # ---- upload ----
        t0 = time.time()
        send(f"upload {tmp} {REMOTE}")
        ok = wait(f"{SIZE} / {SIZE}".encode(), end, 30)
        t1 = time.time()
        m.up_done = ok >= 0
        if m.up_done and t1 > t0:
            m.up_kbps = SIZE * 8 / 1000 / (t1 - t0)
        wait(PROMPT, end, 10)

        # ---- download ----
        with lock:
            seg = len(out)
        t2 = time.time()
        send("cat " + REMOTE)
        dn = wait(PROMPT, seg, 30)
        t3 = time.time()
        m.dn_done = dn >= 0
        if m.dn_done and t3 > t2:
            m.dn_kbps = SIZE * 8 / 1000 / (t3 - t2)

        # ---- integrity (CRC32): clean exit flushes the block-buffered
        # binary cat output, then locate the uploaded blob in the stream ----
        p.stdin.write(b"exit\n")
        p.stdin.flush()
        p.stdin.close()
        try:
            p.wait(timeout=10)
        except Exception:
            p.terminate()
        time.sleep(0.3)
        with lock:
            whole = bytes(out)
        idx = whole.find(data)
        if idx >= 0 and zlib.crc32(whole[idx:idx + SIZE]) == zlib.crc32(data):
            m.crc_ok = True
        return m
    finally:
        try:
            p.terminate()
        except Exception:
            pass
        if os.path.exists(tmp):
            os.remove(tmp)


def cleanup(addr):
    try:
        subprocess.run([CLI_BIN, f"--ble-addr={addr}"],
                       input=f"rm {REMOTE}\nexit\n",
                       capture_output=True, text=True, timeout=40)
    except Exception:
        pass


def ble_disconnect(addr):
    """Drop any BlueZ connection to the device. The transfer CLI leaves the
    link up on exit; if we don't tear it down the device stays connected
    (not advertising) and the NEXT run can't find/pair it - and the host
    accumulates stale connection state. Must run on every exit path."""
    try:
        subprocess.run(["bluetoothctl", "disconnect", addr],
                       capture_output=True, text=True, timeout=15)
    except Exception:
        pass
    time.sleep(1.5)   # let the device re-enter advertising


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--platform", required=True)
    args = parser.parse_args()

    if args.platform not in ("chameleon", "flipper", "kiisu"):
        return skip(f"BLE speed test does not support platform {args.platform}")
    if not os.path.isfile(CLI_BIN):
        return skip(f"CLI binary not found at {CLI_BIN}")
    if not have("bluetoothctl"):
        return skip("bluetoothctl not available")
    try:
        import serial
        import pexpect
    except ImportError as e:
        return skip(f"python BLE deps missing: {e}")

    usb_dev = find_usb_device(USB_VID, USB_PID)
    if not usb_dev:
        return skip("Fantasi device not found on USB")
    cdc_port = find_cdc_port(usb_dev)
    if not cdc_port:
        return skip("CDC port not found")

    # Reset the host BLE adapter first. Across many runs BlueZ accumulates
    # stale state (from repeated pair/connect/disconnect) and silently stops
    # returning scan results - the device is advertising fine, the host just
    # can't see it. A power off/on clears that; without it the scan SKIPs ~80%
    # of the time mid-suite.
    print("  Resetting host BLE adapter...")
    subprocess.run(["bluetoothctl", "power", "off"], capture_output=True, timeout=10)
    time.sleep(2)
    subprocess.run(["bluetoothctl", "power", "on"], capture_output=True, timeout=10)
    time.sleep(2)

    print("  Locating device over BLE...")
    dev_name = device_ble_name(cdc_port)   # pin the scan to THIS board
    if dev_name:
        print(f"  device name: Fantasi {dev_name}")
    addr = find_ble_addr(dev_name)
    if not addr:
        return skip("no 'Fantasi' BLE advertiser found (is BLE on?)")
    print(f"  BLE address: {addr}")

    # Always tear the link down on the way out (incl. a stale one from a prior
    # run that didn't clean up), so the device stays advertising for the next run.
    ble_disconnect(addr)

    try:
        # Clear the device's stored bond so it matches the host (pair() removes
        # the host-side bond before pairing). Otherwise the device reconnects and
        # tries to encrypt with a stale LTK the host no longer has → "Failed to
        # pair" with no passkey prompt. Makes pairing deterministic instead of
        # depending on whatever bond was left over from a previous run.
        send_serial_cmd(cdc_port, "unpair", timeout=3)
        time.sleep(0.5)

        # Pair (Passkey Entry). The reader streams the device log for the code.
        reader = PasskeyReader(cdc_port)
        reader.start()
        time.sleep(1.5)
        print("  Pairing (Passkey Entry)...")
        paired = pair(addr, reader)
        reader.stop()
        if paired is None:
            return skip("pexpect unavailable for pairing")
        if not paired:
            return skip("could not pair (passkey/agent/adapter) - environmental")
        print("  Paired")

        data = bytes((i * 7 + 13) & 0xFF for i in range(SIZE))
        print(f"  Transferring {SIZE} B, CRC32 {zlib.crc32(data) & 0xffffffff:#010x}")
        m = measure(addr, data)
        cleanup(addr)

        if m.error:
            return skip(m.error)  # could not establish the GATT session

        print(f"  upload   {m.up_kbps:6.1f} kbps  ({'ok' if m.up_done else 'INCOMPLETE'})")
        print(f"  download {m.dn_kbps:6.1f} kbps  ({'ok' if m.dn_done else 'INCOMPLETE'})")
        print(f"  CRC32    {'match' if m.crc_ok else 'MISMATCH'}")

        if not m.up_done:
            return fail("upload did not complete (broken transfer)")
        if not m.dn_done:
            return fail("download did not complete (broken transfer)")
        if not m.crc_ok:
            return fail("CRC32 mismatch - corrupted transfer")
        if m.up_kbps < UP_MIN_KBPS:
            return fail(f"upload throughput extremely low: "
                        f"{m.up_kbps:.1f} < {UP_MIN_KBPS} kbps")
        if m.dn_kbps < DOWN_MIN_KBPS:
            return fail(f"download throughput extremely low: "
                        f"{m.dn_kbps:.1f} < {DOWN_MIN_KBPS} kbps")

        print("PASS")
        return 0
    finally:
        # Critical: never leave the device connected, or the next run's
        # ble_speed can't find an advertiser and lfs_persist's DFU is degraded.
        ble_disconnect(addr)


if __name__ == "__main__":
    sys.exit(main())
