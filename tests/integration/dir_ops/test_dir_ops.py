#!/usr/bin/env python3
"""Directory + file operations across every transport and filesystem.

Runs a common battery of file operations over each transport the device offers
(serial/MSC, WebUSB, and BLE when available) and each filesystem present on the
platform, checking the results survive a *fresh* session - which, over serial,
forces the write back through the synthetic-FAT bridge (an in-session read would
be served from the host's page cache and never touch the device).

Coverage per (transport, filesystem):
  - create + read-back content + listing
  - same-filesystem rename (mv) - content preserved, old name gone, nothing else lost
  - same-filesystem copy (cp) - both names present with the same content
  - subdirectories where the FS supports them (/ramfs is flat and is skipped)
  - `..` traversal (cd in, cd .., resolve back to the mount root)
  - rm / rmdir actually remove, and remove *only* the target

Plus cross-filesystem move and copy between the internal FS and /ramfs.

The invariant checked throughout: a file only disappears when it is rm'd.
"""

import argparse
import os
import re
import subprocess
import sys
import uuid

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.device import (
    PLATFORMS, USB_VID, USB_PID,
    find_usb_device, find_cdc_port, CLI_WEBUSB_SENTINEL,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")


def _strip(s):
    return re.sub(r'\033\[[0-9;]*m', '', s)


class Transport:
    """One way to reach the device; `.args` is the CLI prefix that selects it.

    `piped` picks how a multi-command script is delivered. The WebUSB/BLE protobuf
    transports run a script fed on stdin (fast: one connection). Interactive serial
    needs a TTY to pump the link (rl_poll_serial), which a pipe isn't, so serial
    runs one `-c` command per invocation - each a fresh mount, which is exactly what
    forces writes back through the synthetic-FAT bridge rather than the page cache.
    """
    def __init__(self, name, args, piped):
        self.name = name
        self.args = args
        self.piped = piped

    def _cmds(self, script):
        return [c.strip() for c in script.strip().splitlines()
                if c.strip() and c.strip() != "exit"]

    def run(self, script, timeout=120):
        if self.piped:
            r = subprocess.run(
                [CLI_BIN, *self.args],
                input="\n".join(self._cmds(script)) + "\nexit\n",
                capture_output=True, text=True, timeout=timeout,
            )
            return _strip(r.stdout + r.stderr)
        out = ""
        for cmd in self._cmds(script):
            r = subprocess.run(
                [CLI_BIN, *self.args, "-c", cmd],
                capture_output=True, text=True, timeout=timeout,
            )
            out += _strip(r.stdout + r.stderr) + "\n"
        return out

    def ls(self, path):
        """Set of entry basenames in `path` (handles both FAT 'name/' and proto
        'name  <dir>' listing formats)."""
        out = self.run(f"ls {path}\nexit\n")
        names = set()
        for line in out.splitlines():
            line = line.strip()
            if not line or line.startswith(("transport:", "Unmounted", "fantasi",
                                            "cannot", "error", "no ")):
                continue
            tok = line.split()[0]
            names.add(tok.rstrip("/"))
        return names

    def cat(self, path):
        out = self.run(f"cat {path}\nexit\n")
        return out


def discover_filesystems(t):
    """Return the writable filesystem roots the device reports via `df`."""
    out = t.run("df\nexit\n")
    fses = []
    for line in out.splitlines():
        m = re.match(r'\s*(/\S*)\s', line)
        if not m:
            continue
        p = m.group(1)
        if p in ("/", "/ramfs") or p.startswith("/mnt/ext"):
            fses.append(p)
    # Stable, predictable order: internal first, then ramfs, then ext mounts.
    order = {"/": 0, "/ramfs": 1}
    return sorted(set(fses), key=lambda p: (order.get(p, 2), p))


# A filesystem is "flat" (no subdirectories) if it's the RAM overlay.
def supports_subdirs(fs):
    return fs != "/ramfs"


class Fail(Exception):
    pass


def check(cond, msg):
    if not cond:
        raise Fail(msg)


def battery(m, v, fses):
    """Run the operation battery, mutating over transport `m` and verifying over
    `v`. `v` is always a protobuf transport (WebUSB/BLE), which reads the device's
    VFS directly - the ground truth - so checks never race the host's page cache of
    the synthetic MSC drive (which lags after another transport writes)."""
    tag = uuid.uuid4().hex[:8]
    content = f"dir-ops-{tag}\n"
    local = os.path.join(REPO_ROOT, "build", f"dir_ops_{tag}.txt")
    with open(local, "w") as f:
        f.write(content)

    def base(fs, name):
        return f"{fs.rstrip('/')}/{name}" if fs != "/" else f"/{name}"

    try:
        for fs in fses:
            print(f"    [{m.name}] filesystem {fs}")
            root = base(fs, f"do_{tag}") if supports_subdirs(fs) else fs
            a = f"{root.rstrip('/')}/a.txt"
            b = f"{root.rstrip('/')}/b.txt"
            c = f"{root.rstrip('/')}/c.txt"

            # --- create + read-back ---
            setup = ""
            if supports_subdirs(fs):
                setup += f"mkdir {root}\n"
            setup += f"upload {local} {a}\nexit\n"
            m.run(setup)
            check(content.strip() in v.cat(a),
                  f"{fs}: created file did not read back with its content")
            check("a.txt" in v.ls(root), f"{fs}: created file not listed")

            # --- same-FS rename: a -> b (content kept, a gone) ---
            m.run(f"mv {a} {b}\nexit\n")
            names = v.ls(root)
            check("b.txt" in names, f"{fs}: rename lost the destination (b.txt)")
            check("a.txt" not in names, f"{fs}: rename left the source (a.txt)")
            check(content.strip() in v.cat(b), f"{fs}: rename lost the file content")

            # --- same-FS copy: b -> c (both present) ---
            m.run(f"cp {b} {c}\nexit\n")
            names = v.ls(root)
            check("b.txt" in names and "c.txt" in names,
                  f"{fs}: copy did not leave both source and destination")
            cb, cc = v.cat(b), v.cat(c)
            check(content.strip() in cb and content.strip() in cc,
                  f"{fs}: copy produced wrong content (b={cb!r} c={cc!r})")

            # --- subdirectory + `..` traversal (FSes that support subdirs) ---
            if supports_subdirs(fs):
                sub = f"{root.rstrip('/')}/sub"
                g = f"{sub}/g.txt"
                m.run(f"mkdir {sub}\nupload {local} {g}\nexit\n")
                check("g.txt" in v.ls(sub), f"{fs}: file in nested subdir not listed")
                # `..` via path normalisation: /root/sub/.. resolves to /root.
                check("b.txt" in v.ls(f"{sub}/.."),
                      f"{fs}: '..' did not resolve to the parent dir")
                check(content.strip() in v.cat(f"{sub}/../b.txt"),
                      f"{fs}: read through '..' failed")
                m.run(f"rm {g}\nrmdir {sub}\nexit\n")
                check("sub" not in v.ls(root), f"{fs}: rmdir left the subdir behind")

            # --- rm removes only the target ---
            m.run(f"rm {c}\nexit\n")
            names = v.ls(root)
            check("c.txt" not in names, f"{fs}: rm did not remove the file")
            check("b.txt" in names, f"{fs}: rm removed an unrelated file (b.txt)")
            # leave `b` for the cross-FS phase

        # --- cross-filesystem move + copy between internal / and /ramfs ---
        if "/" in fses and "/ramfs" in fses:
            print(f"    [{m.name}] cross-FS  /  <->  /ramfs")
            src = f"{base('/', f'do_{tag}')}/b.txt"    # left over from the / battery
            xmv, xcp = "/ramfs/xmv.txt", "/ramfs/xcp.txt"
            m.run(f"cp {src} {xcp}\nexit\n")
            check(content.strip() in v.cat(xcp), "cross-FS copy content wrong")
            check(content.strip() in v.cat(src), "cross-FS copy lost the source")
            m.run(f"mv {src} {xmv}\nexit\n")
            check(content.strip() in v.cat(xmv), "cross-FS move content wrong")
            check("b.txt" not in v.ls(base("/", f"do_{tag}")),
                  "cross-FS move left the source")
            m.run(f"rm {xmv}\nrm {xcp}\nexit\n")

        # --- teardown ---
        for fs in fses:
            r = base(fs, f"do_{tag}") if supports_subdirs(fs) else fs.rstrip("/")
            m.run(f"rm {r}/a.txt\nrm {r}/b.txt\nrm {r}/c.txt\n"
                  + (f"rmdir {r}\n" if supports_subdirs(fs) else "") + "exit\n")
            if supports_subdirs(fs):
                check(f"do_{tag}" not in v.ls(fs), f"{fs}: scratch dir survived teardown")
    finally:
        os.remove(local)


def cross_transport(ts):
    """Interleave transports within one operation sequence: create on one, rename on
    another, delete on a third - checking after each step that *every* transport sees
    the same live filesystem. This is what proves the views never diverge (the whole
    point of stable, host-consistent clusters)."""
    if len(ts) < 2:
        print("  (single transport - skipping cross-transport consistency)")
        return
    tag = uuid.uuid4().hex[:8]
    content = f"xt-{tag}\n"
    local = os.path.join(REPO_ROOT, "build", f"xt_{tag}.txt")
    with open(local, "w") as f:
        f.write(content)
    n1, n2 = f"xt_{tag}_1.txt", f"xt_{tag}_2.txt"
    p1, p2 = "/" + n1, "/" + n2
    try:
        # created on ts[0] -> visible with correct content on every other transport
        ts[0].run(f"upload {local} {p1}\nexit\n")
        for t in ts[1:]:
            check(content.strip() in t.cat(p1),
                  f"{t.name} did not see the file created over {ts[0].name}")
        # renamed on ts[1] -> every transport sees the new name, not the old, right content
        ts[1].run(f"mv {p1} {p2}\nexit\n")
        for t in ts:
            names = t.ls("/")
            check(n2 in names, f"{t.name} did not see the rename done over {ts[1].name}")
            check(n1 not in names, f"{t.name} still shows the old name after {ts[1].name} renamed it")
            check(content.strip() in t.cat(p2),
                  f"{t.name} sees wrong content after the cross-transport rename")
        # deleted on the last transport -> gone everywhere
        ts[-1].run(f"rm {p2}\nexit\n")
        for t in ts:
            check(n2 not in t.ls("/"), f"{t.name} still shows the file after {ts[-1].name} rm'd it")
        print(f"  cross-transport consistency OK across {[t.name for t in ts]}")
    finally:
        os.remove(local)


def find_ble_addr(usb_dev):
    """The BLE peripheral advertises as 'Fantasi <name>' where <name> is the same
    device name as the USB iSerial - so match on that to reach the *same* device
    (not some other bonded Fantasi). Returns its BD address, or None."""
    try:
        name = open(os.path.join(usb_dev, "serial")).read().strip()
    except OSError:
        return None
    try:
        subprocess.run(["bluetoothctl", "--timeout", "8", "scan", "on"],
                       capture_output=True, timeout=15)
        out = subprocess.run(["bluetoothctl", "devices"],
                             capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for line in out.splitlines():
        m = re.match(r"Device (\S+) Fantasi (.+)", line)
        if m and m.group(2).strip() == name:
            return m.group(1)
    return None


def available_transports(platform):
    """Transports to exercise: serial always; WebUSB and BLE when reachable."""
    ts = []
    usb_dev = find_usb_device(USB_VID, USB_PID)
    if not usb_dev:
        return ts, "device not found"
    cdc = find_cdc_port(usb_dev)
    if cdc and cdc != CLI_WEBUSB_SENTINEL:
        ts.append(Transport("serial", ["--serial", cdc], piped=False))
    ts.append(Transport("usb", ["--usb"], piped=True))
    # BLE is optional: include it only if we can find and reach the same device.
    addr = find_ble_addr(usb_dev)
    if addr:
        ble = Transport("ble", ["--ble-addr=" + addr], piped=True)
        probe = ble.run("version\n", timeout=45)
        if "transport: BLE" in probe and "fantasi" in probe:
            ts.append(ble)
        else:
            print(f"  (BLE device {addr} found but connect failed - skipping)")
    else:
        print("  (BLE not available - skipping that transport)")
    return ts, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", required=True)
    args = ap.parse_args()

    if args.platform not in PLATFORMS:
        print(f"FAIL: unknown platform {args.platform}")
        return 1
    if not os.path.isfile(CLI_BIN):
        print(f"SKIP: CLI binary not found at {CLI_BIN}")
        return 77

    transports, err = available_transports(args.platform)
    if err:
        print(f"FAIL: {err}")
        return 1

    # Verify over a protobuf transport (WebUSB) - it reads the device VFS directly,
    # so checks see the ground truth rather than the host's cached MSC view.
    verify = next((t for t in transports if t.name == "usb"), transports[0])

    # Filesystems are a property of the device, not the transport; discover once.
    fses = discover_filesystems(verify)
    print(f"  transports: {[t.name for t in transports]}")
    print(f"  filesystems: {fses}")
    if not fses:
        print("FAIL: no filesystems reported by df")
        return 1

    try:
        for t in transports:
            print(f"  === transport: {t.name} ===")
            battery(t, verify, fses)
        print("  === cross-transport consistency ===")
        cross_transport(transports)
    except Fail as e:
        print(f"FAIL: {e}")
        return 1
    except subprocess.TimeoutExpired:
        print("FAIL: CLI timed out")
        return 1

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
