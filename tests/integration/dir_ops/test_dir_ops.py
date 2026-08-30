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
import glob
import os
import re
import subprocess
import sys
import uuid
import zlib

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


def find_msc_mount():
    """Return the connected Fantasi MSC block device and mountpoint."""
    usb_dev = find_usb_device(USB_VID, USB_PID)
    blocks = glob.glob(os.path.join(
        usb_dev, "*", "host*", "target*", "*:*:*:*", "block", "sd*"))
    check(bool(blocks), "composite MSC block device was not found")
    block = "/dev/" + os.path.basename(blocks[0])
    r = subprocess.run(
        ["findmnt", "-rn", "-S", block, "-o", "TARGET"],
        capture_output=True, text=True, timeout=10,
    )
    mounts = [line.strip() for line in r.stdout.splitlines() if line.strip()]
    check(r.returncode == 0 and mounts,
          f"composite MSC block {block} is not mounted ({r.stderr!r})")
    return block, mounts[0]


def battery(m, v, fses):
    """Run the operation battery, mutating over transport `m` and verifying over
    `v`. `v` is always a protobuf transport (WebUSB/BLE), which reads the device's
    VFS directly - the ground truth - so checks never race the host's page cache of
    the synthetic MSC drive (which lags after another transport writes)."""
    tag = uuid.uuid4().hex[:8]
    content = f"dir-ops-{tag}\n"
    local = os.path.join(REPO_ROOT, "build", f"dir_ops_{tag}.txt")
    xmv = f"/ramfs/xmv_{tag}.txt"
    xcp = f"/ramfs/xcp_{tag}.txt"
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

            # FAT short entries carry lowercase state in NTRes byte 12. Exercise
            # an uppercase 8.3 rename, then an LFN rename, with each serial command
            # in a fresh MSC process so cached alias identity cannot hide a bug.
            if m.name == "serial" and fs == "/":
                upper = f"{root.rstrip('/')}/UPPER.TXT"
                mixed = f"{root.rstrip('/')}/MiXeD.txt"
                m.run(f"mv {b} {upper}\nexit\n")
                names = v.ls(root)
                check("UPPER.TXT" in names and "b.txt" not in names,
                      "/: uppercase FAT rename lost the exact destination case")
                m.run(f"mv {upper} {mixed}\nexit\n")
                names = v.ls(root)
                check("MiXeD.txt" in names and "UPPER.TXT" not in names,
                      "/: mixed-case LFN rename was not preserved")
                m.run(f"mv {mixed} {b}\nexit\n")
                check(content.strip() in v.cat(b),
                      "/: case/LFN rename round trip lost content")

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

                if m.name == "serial" and fs == "/":
                    # Place one long entry across a 512-byte directory-sector
                    # boundary, then rename its neighbour in a fresh MSC mount.
                    # The rewritten sector contains the target SFN but not all
                    # of its LFN fragments; it must retain the long VFS name.
                    split = f"{root.rstrip('/')}/lfn_split"
                    seeded = [f"a{i}.txt" for i in range(5)]
                    seeded += ["b_padding_long.txt", "m_preserve_long.txt", "z.txt"]
                    # Seed through the VFS so the next synthetic FAT snapshot gives
                    # every short-looking name an LFN+SFN pair. Seeding through MSC
                    # would leave aN.txt as a native one-slot entry and the claimed
                    # sector-boundary layout would not actually exist.
                    v.run(f"mkdir {split}\n" +
                          "".join(f"upload {local} {split}/{name}\n" for name in seeded) +
                          "exit\n")
                    try:
                        m.run(f"mv {split}/z.txt {split}/y.txt\nexit\n")
                        split_names = v.ls(split)
                        check("m_preserve_long.txt" in split_names,
                              "/: split LFN rewrite lost the preserved long name")
                        check(not any(name.startswith("FAP") for name in split_names),
                              "/: split LFN rewrite exposed a synthetic alias")
                        check(content.strip() in v.cat(f"{split}/m_preserve_long.txt"),
                              "/: split LFN rewrite lost file content")
                    finally:
                        # Everything beneath this unique fixture is test-owned.
                        leftovers = v.ls(split)
                        if leftovers:
                            v.run("".join(f"rm {split}/{name}\n" for name in leftovers) +
                                  f"rmdir {split}\nexit\n")
                        else:
                            v.run(f"rmdir {split}\nexit\n")

                    # Two specials + five two-slot names + one three-slot name =
                    # 15 entries. The next 17-character leaf starts in slot 15, so
                    # its first LFN fragment is in one sector while its remaining
                    # fragment and SFN are in the next. Linux writes those sectors
                    # in either order; both must reconstruct the exact long name.
                    create_split = f"{root.rstrip('/')}/lfn_create"
                    create_seeded = [f"a{i}.txt" for i in range(5)]
                    create_seeded += ["b_padding_long.txt"]
                    long_created = "x_create_long.txt"
                    v.run(f"mkdir {create_split}\n" +
                          "".join(f"upload {local} {create_split}/{name}\n"
                                  for name in create_seeded) +
                          "exit\n")
                    try:
                        create_out = m.run(
                            f"upload {local} {create_split}/{long_created}\nexit\n")
                        create_names = v.ls(create_split)
                        create_read = v.cat(f"{create_split}/{long_created}")
                        check(long_created in create_names,
                              "/: split LFN create lost the exact long name "
                              f"(command={create_out!r}, names={sorted(create_names)!r})")
                        check(not any("~" in name or name.startswith("FAP")
                                      for name in create_names),
                              "/: split LFN create exposed an 8.3 alias "
                              f"(names={sorted(create_names)!r})")
                        check(content.strip() in create_read,
                              "/: split LFN create lost file content "
                              f"(read={create_read!r})")
                    finally:
                        leftovers = v.ls(create_split)
                        if leftovers:
                            v.run("".join(f"rm {create_split}/{name}\n"
                                          for name in leftovers) +
                                  f"rmdir {create_split}\nexit\n")
                        else:
                            v.run(f"rmdir {create_split}\nexit\n")

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
            copy_out = m.run(f"cp {src} {xcp}\nexit\n")
            copied = v.cat(xcp)
            check(content.strip() in copied,
                  f"cross-FS copy content wrong (command={copy_out!r}, read={copied!r})")
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
        # Unique cross-FS names prevent an interrupted transport from poisoning
        # the next one. Remove them through the verifier as a best-effort fallback
        # when the mutating transport itself was what failed.
        try:
            v.run(f"rm {xmv}\nrm {xcp}\nexit\n")
        except (OSError, subprocess.SubprocessError):
            pass
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
        create_out = ts[0].run(f"upload {local} {p1}\nexit\n")
        for t in ts[1:]:
            created_read = t.cat(p1)
            if content.strip() not in created_read:
                raise Fail(
                    f"{t.name} did not see the file created over {ts[0].name} "
                    f"(command={create_out!r}, read={created_read!r}, "
                    f"root={sorted(t.ls('/'))!r})")
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
        # Best-effort cleanup also covers a failure before the normal delete step.
        cleaner = next((t for t in ts if t.piped), ts[0])
        try:
            cleaner.run(f"rm {p1}\nrm {p2}\nexit\n")
        except (OSError, subprocess.SubprocessError):
            pass
        os.remove(local)


def msc_presync_newdir(serial, verify):
    """Populate a new MSC directory before its parent entry is synced.

    CLI mkdir/upload commands each synchronize, so they cannot reproduce Linux
    writing a child directory sector before firmware has decoded the parent entry.
    Seed seven synthetic two-slot names after dot/dot (next SFN = sector slot 16),
    mutate the already-mounted FAT directly, then issue one explicit CLI sync.
    """
    tag = uuid.uuid4().hex[:6]
    parents = (f"/kdl_{tag}", f"/kdu_{tag}")
    marker = f"/kd_{tag}.trg"
    payloads = (("child", f"lower-{tag}\n"), ("CHILD", f"upper-{tag}\n"))
    large_parent = f"/kde_{tag}"
    large_block = None
    local = os.path.join(REPO_ROOT, "build", f"kd_{tag}.txt")
    with open(local, "w") as f:
        f.write(f"seed-{tag}\n")

    try:
        setup = []
        for parent in parents:
            setup.append(f"mkdir {parent}")
            setup.extend(f"upload {local} {parent}/a{i}.txt" for i in range(7))
        setup.append(f"upload {local} {marker}")
        verify.run("\n".join(setup) + "\nexit\n")

        # Force the serial/MSC path to refresh and mount the just-seeded layout.
        serial.run(f"ls {parents[0]}\nexit\n")
        _, mountpoint = find_msc_mount()

        # Deliberately no fsync/syncfs between mkdir and child creation.
        for parent, (child, payload) in zip(parents, payloads):
            raw_dir = os.path.join(mountpoint, parent.lstrip("/"), child)
            os.mkdir(raw_dir)
            with open(os.path.join(raw_dir, "file.txt"), "wb") as f:
                f.write(payload.encode())

        # cmd_rm performs syncfs + SCSI SYNCHRONIZE CACHE for the shared mount.
        sync_out = serial.run(f"rm {marker}\nexit\n")
        for parent, (child, payload) in zip(parents, payloads):
            names = verify.ls(f"{parent}/{child}")
            readback = verify.cat(f"{parent}/{child}/file.txt")
            check("file.txt" in names and payload.strip() in readback,
                  "MSC pre-sync child directory write was lost "
                  f"({parent}/{child}, sync={sync_out!r}, "
                  f"names={sorted(names)!r}, read={readback!r})")
        print("  MSC pre-sync new-directory recovery OK")

        # Exercise the normal desktop unmount boundary with a child file larger
        # than one reported FAT allocation unit.
        verify.run(f"upload {local} {marker}\nexit\n")
        serial.run("ls /\nexit\n")
        large_block, mountpoint = find_msc_mount()
        cluster_bytes = os.statvfs(mountpoint).f_frsize
        check(cluster_bytes >= 512,
              "MSC mount reported invalid cluster geometry")
        seed = sum(tag.encode()) & 0xff
        initial_payload = bytes((seed + i * 37) & 0xff
                                for i in range(cluster_bytes + 137))
        large_payload = bytes((seed + 19 + i * 41) & 0xff
                              for i in range(cluster_bytes + 649))
        raw_dir = os.path.join(mountpoint, large_parent.lstrip("/"))
        os.mkdir(raw_dir)
        pending_path = os.path.join(raw_dir, "pending.bin")
        payload_path = os.path.join(raw_dir, "payload.bin")
        deleted_path = os.path.join(raw_dir, "deleted.bin")
        with open(pending_path, "wb") as f:
            f.write(initial_payload)
        r = subprocess.run(["sync", "-f", mountpoint], timeout=30)
        check(r.returncode == 0, "syncfs failed before pending-file rename")
        os.rename(pending_path, payload_path)
        with open(deleted_path, "wb") as f:
            f.write(initial_payload[:1024])
        r = subprocess.run(["sync", "-f", mountpoint], timeout=30)
        check(r.returncode == 0, "syncfs failed before pending-file delete")
        os.unlink(deleted_path)
        r = subprocess.run(["sync", "-f", mountpoint], timeout=30)
        check(r.returncode == 0, "syncfs failed after pending-file delete")
        sync1_out = serial.run(f"rm {marker}\nexit\n")

        path = f"{large_parent}/payload.bin"
        names = verify.ls(large_parent)
        crc_out = verify.run(f"crc32 {path}\nexit\n")
        initial = (f"{zlib.crc32(initial_payload) & 0xffffffff:08x} "
                   f"{len(initial_payload)} {path}")
        check("payload.bin" in names and "pending.bin" not in names and
              "deleted.bin" not in names and initial in crc_out,
              "MSC pending rename/delete reconciliation failed "
              f"(sync={sync1_out!r}, names={sorted(names)!r}, crc={crc_out!r})")

        with open(payload_path, "wb") as f:
            f.write(large_payload)
        r = subprocess.run(
            ["udisksctl", "unmount", "-b", large_block],
            capture_output=True, text=True, timeout=30,
        )
        check(r.returncode == 0,
              f"MSC multi-cluster unmount failed ({r.stderr!r})")
        large_block = None
        names = verify.ls(large_parent)
        crc_out = verify.run(f"crc32 {path}\nexit\n")
        expected = f"{zlib.crc32(large_payload) & 0xffffffff:08x} {len(large_payload)} {path}"
        check("payload.bin" in names and "deleted.bin" not in names and
              expected in crc_out,
              "MSC unmount lost a multi-cluster overwrite in a new directory "
              f"(names={sorted(names)!r}, crc={crc_out!r})")
        print("  MSC multi-cluster new-directory recovery OK")
    finally:
        # Best effort: first make any raw host writes visible, then remove only
        # this test's unique fixture through the VFS ground-truth transport.
        try:
            serial.run(f"rm {marker}\nexit\n")
        except (OSError, subprocess.SubprocessError):
            pass
        if large_block:
            subprocess.run(
                ["udisksctl", "unmount", "-b", large_block],
                capture_output=True, timeout=30,
            )
        cleanup = []
        for parent, (child, _) in zip(parents, payloads):
            cleanup.append(f"rm {parent}/{child}/file.txt")
            cleanup.append(f"rmdir {parent}/{child}")
            cleanup.extend(f"rm {parent}/a{i}.txt" for i in range(7))
            cleanup.append(f"rmdir {parent}")
        cleanup.append(f"rm {large_parent}/payload.bin")
        cleanup.append(f"rm {large_parent}/pending.bin")
        cleanup.append(f"rm {large_parent}/deleted.bin")
        cleanup.append(f"rmdir {large_parent}")
        cleanup.append(f"rm {marker}")
        try:
            verify.run("\n".join(cleanup) + "\nexit\n")
        except (OSError, subprocess.SubprocessError):
            pass
        os.remove(local)


def msc_newdir_reclaim(serial, verify, raw_mount):
    """Reuse reconciled routes and preserve them across a directory rename."""
    tag = uuid.uuid4().hex[:6]
    root = f"/nr{tag}"
    dirs = [f"{root}/d{i:02d}" for i in range(12)]
    content = f"newdir-reclaim-{tag}\n"
    local = os.path.join(REPO_ROOT, "build", f"nr_{tag}.txt")
    raw_old = dirs[4]
    raw_renamed = f"{root}/r04"
    raw_name = "after.txt"
    raw_marker = f"{root}/sync.trg"
    raw_block = None
    with open(local, "w") as f:
        f.write(content)

    try:
        # One CLI process keeps one MSC mount alive. Each mkdir synchronizes,
        # so its transient route should be available for the next directory.
        commands = [f"mkdir {root}"]
        for path in dirs:
            commands.append(f"mkdir {path}")
            commands.append(f"upload {local} {path}/f.txt")
        old = dirs[3]
        renamed = f"{root}/r03"
        nested = f"{dirs[11]}/sub"
        extended = [f"x{i:02d}.txt" for i in range(15)]
        commands.extend(f"cp {old}/f.txt {old}/{name}" for name in extended)
        commands.extend((
            f"mv {old} {renamed}",
            f"mv {renamed}/f.txt {renamed}/g.txt",
            f"upload {local} {renamed}/after.txt",
            f"mkdir {nested}",
            f"upload {local} {nested}/n.txt",
            f"upload {local} {raw_marker}",
        ))
        out = serial.run(";".join(commands) + "\nexit\n", timeout=180)

        names = verify.ls(root)
        expected = [renamed if path == old else path for path in dirs]
        missing = [os.path.basename(path) for path in expected
                   if os.path.basename(path) not in names]
        check(not missing,
              "MSC new-directory routes were exhausted "
              f"(missing={missing!r}, command={out!r})")
        check(os.path.basename(old) not in names,
              "MSC directory rename left its source behind "
              f"(names={sorted(names)!r}, command={out!r})")
        for path in expected:
            child_names = verify.ls(path)
            leaf = "g.txt" if path == renamed else "f.txt"
            readback = verify.cat(f"{path}/{leaf}")
            check(leaf in child_names and content.strip() in readback,
                  "MSC write after new-directory reconciliation was lost "
                  f"({path}, names={sorted(child_names)!r}, read={readback!r}, "
                  f"command={out!r})")
        renamed_names = verify.ls(renamed)
        missing_extended = [name for name in extended if name not in renamed_names]
        after_read = verify.cat(f"{renamed}/after.txt")
        check(not missing_extended and "after.txt" in renamed_names and
              content.strip() in after_read,
              "MSC directory-extension route did not survive a rename "
              f"(missing={missing_extended!r}, names={sorted(renamed_names)!r}, "
              f"read={after_read!r}, command={out!r})")
        nested_names = verify.ls(nested)
        nested_read = verify.cat(f"{nested}/n.txt")
        check("n.txt" in nested_names and content.strip() in nested_read,
              "MSC directory rename corrupted a later nested-directory route "
              f"(names={sorted(nested_names)!r}, read={nested_read!r}, "
              f"command={out!r})")

        if raw_mount:
            # Operate on the continuously mounted filesystem so Linux can publish
            # a moved directory's destination before its deleted source.
            serial.run(f"ls {raw_old}\nexit\n")
            raw_block, mountpoint = find_msc_mount()
            old_host = os.path.join(mountpoint, raw_old.lstrip("/"))
            renamed_host = os.path.join(mountpoint, raw_renamed.lstrip("/"))
            os.rename(old_host, renamed_host)
            with open(os.path.join(renamed_host, raw_name), "w") as f:
                f.write(content)
            r = subprocess.run(["sync", "-f", mountpoint], timeout=30)
            check(r.returncode == 0, "syncfs failed after raw MSC directory rename")
            sync_out = serial.run(f"rm {raw_marker}\nexit\n")
            check("filesystem sync failed" not in sync_out,
                  f"SCSI sync failed after raw directory rename ({sync_out!r})")
            r = subprocess.run(
                ["udisksctl", "unmount", "-b", raw_block],
                capture_output=True, text=True, timeout=30,
            )
            check(r.returncode == 0,
                  f"MSC unmount failed after raw directory rename ({r.stderr!r})")
            raw_block = None

            root_names = verify.ls(root)
            renamed_names = verify.ls(raw_renamed)
            check(os.path.basename(raw_old) not in root_names and
                  os.path.basename(raw_renamed) in root_names and
                  {"f.txt", raw_name}.issubset(renamed_names),
                  "mounted MSC directory rename failed "
                  f"(root={sorted(root_names)!r}, names={sorted(renamed_names)!r})")
            check(content.strip() in verify.cat(f"{raw_renamed}/{raw_name}"),
                  "mounted MSC write after directory rename lost file content")
        print("  MSC new-directory route reclamation OK")
    finally:
        if raw_block:
            subprocess.run(
                ["udisksctl", "unmount", "-b", raw_block],
                capture_output=True, timeout=30,
            )
        cleanup = []
        for path in (dirs[3], f"{root}/r03"):
            cleanup.append(f"rm {path}/f.txt")
            cleanup.append(f"rm {path}/g.txt")
            cleanup.append(f"rm {path}/n.txt")
            cleanup.append(f"rm {path}/after.txt")
            cleanup.extend(f"rm {path}/{name}" for name in extended)
        cleanup.append(f"rm {dirs[11]}/sub/n.txt")
        cleanup.append(f"rmdir {dirs[11]}/sub")
        cleanup.append(f"rm {raw_renamed}/f.txt")
        cleanup.append(f"rm {raw_renamed}/{raw_name}")
        cleanup.append(f"rm {raw_marker}")
        cleanup.append(f"rmdir {raw_renamed}")
        for path in dirs:
            cleanup.append(f"rm {path}/f.txt")
            cleanup.append(f"rmdir {path}")
        cleanup.append(f"rmdir {root}/r03")
        cleanup.append(f"rmdir {root}")
        try:
            verify.run("\n".join(cleanup) + "\nexit\n")
        except (OSError, subprocess.SubprocessError):
            pass
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
        # Empty listings and failed opens must remain distinct on stateful
        # protobuf transports. Check before filesystem mutation begins.
        for t in (x for x in transports if x.piped):
            missing = f"/__fantasi_missing_{uuid.uuid4().hex}"
            out = t.run(f"cd /\ncd {missing}\npwd\nls {missing}\nexit\n")
            check(f"not a directory: {missing}" in out,
                  f"{t.name}: cd accepted a nonexistent directory ({out!r})")
            check(f"error: not a directory" in out,
                  f"{t.name}: ls did not reject a nonexistent directory ({out!r})")
            check(any(line.strip() == "/" for line in out.splitlines()),
                  f"{t.name}: failed cd changed the working directory ({out!r})")

        for t in transports:
            print(f"  === transport: {t.name} ===")
            battery(t, verify, fses)
        serial = next((t for t in transports if t.name == "serial"), None)
        if serial:
            print("  === MSC new-directory route reclamation ===")
            msc_newdir_reclaim(
                serial, verify,
                PLATFORMS[args.platform]["msc_mode"] == "composite",
            )
        if PLATFORMS[args.platform]["msc_mode"] == "composite":
            if serial:
                print("  === MSC pre-sync directory ordering ===")
                msc_presync_newdir(serial, verify)
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
