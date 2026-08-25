#!/usr/bin/env python3
"""Exercise independent, concurrent WebUSB protobuf sessions.

This intentionally launches separate host processes: no broker, lock file, or
session owner is shared between them. It covers idle-session accounting/RAM,
a blocked cancellable stream alongside a short command, a burst of commands,
and two simultaneous multi-frame uploads.
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
ANSI = re.compile(r"\033\[[0-9;]*m")
UPTIME = re.compile(
    r"^(?:(\d+)d )?(?:(\d+)h )?(?:(\d+)m )?(\d+\.\d+)s$")


def cli(command, timeout=30):
    return subprocess.run([CLI_BIN, "--usb", "-c", command],
                          capture_output=True, text=True, timeout=timeout)


def clean(text):
    return ANSI.sub("", text)


def session_count(output):
    m = re.search(r"(\d+) protobuf sessions?", clean(output))
    return int(m.group(1)) if m else None


def free_bytes(output):
    m = re.search(r"heap:\s*(\d+)/", clean(output))
    return int(m.group(1)) if m else None


def device_lines(output):
    """Return command output without host transport/switch banners."""
    return [line.strip() for line in clean(output).splitlines()
            if line.strip() and
            not line.strip().startswith(("transport:", "switching to WebUSB"))]


def uptime_values(output):
    values = []
    for line in clean(output).replace("\r", "").splitlines():
        m = UPTIME.fullmatch(line.strip())
        if not m:
            continue
        days, hours, minutes, seconds = m.groups()
        values.append((int(days or 0) * 86400 + int(hours or 0) * 3600 +
                       int(minutes or 0) * 60 + float(seconds)))
    return values


def fail(message, detail=""):
    print(f"FAIL: {message}")
    if detail:
        print(detail[:1000])
    return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", required=True)
    platform = ap.parse_args().platform
    if platform not in PLATFORMS:
        return fail(f"unknown platform {platform}")
    if not os.path.isfile(CLI_BIN):
        print(f"SKIP: CLI binary not found at {CLI_BIN}")
        return 77
    if not find_usb_device(USB_VID, USB_PID):
        return fail("Fantasi USB device not found")

    # Every target must negotiate the same stateless READ contract. If OPEN and
    # READ ever drift to different semantics, this one response is replayed
    # until the mailbox lease expires and makes WebUSB appear to connect slowly.
    started = time.monotonic()
    probe = cli("whoami", timeout=15)
    elapsed = time.monotonic() - started
    if probe.returncode or len(device_lines(probe.stdout)) != 1:
        return fail("initial WebUSB response was lost or replayed",
                    probe.stdout + probe.stderr)
    if elapsed > 8.0:
        return fail(f"initial WebUSB response was delayed ({elapsed:.2f}s)",
                    probe.stdout + probe.stderr)
    print(f"  initial WebUSB response: one frame in {elapsed:.2f}s")

    # ps produces about 500 characters through many cli_write() calls. Device
    # uptimes around it exclude USB discovery and session-open time.
    paced = cli("uptime;ps;uptime", timeout=15)
    stamps = uptime_values(paced.stdout)
    if paced.returncode or len(stamps) != 2 or "STACKFREE" not in paced.stdout:
        return fail("could not measure complete WebUSB ps output",
                    paced.stdout + paced.stderr)
    ps_span = stamps[1] - stamps[0]
    # SAM7S reads the response in seven-byte EP0 controls, so its bound is more
    # generous than targets with a normal EP0 size.
    ps_limit = 2.0 if platform == "proxmark3" else 0.5
    if ps_span > ps_limit:
        return fail(f"WebUSB ps output was paced line-by-line ({ps_span:.3f}s)",
                    paced.stdout)
    print(f"  500-byte ps response: {ps_span * 1000:.0f} ms")

    # The first --usb invocation also switches a switch-mode board to WebUSB.
    base_w = cli("w", timeout=40)
    base_count = session_count(base_w.stdout)
    if base_w.returncode or base_count is None:
        return fail("could not establish the baseline WebUSB session", base_w.stdout + base_w.stderr)
    time.sleep(0.5)  # let the just-deleted one-shot worker be reclaimed
    base_free_run = cli("free")
    base_free = free_bytes(base_free_run.stdout)
    if base_free is None:
        return fail("could not read baseline heap", base_free_run.stdout + base_free_run.stderr)

    holders = []
    holder_count = 4
    try:
        # Keep stdin open but silent. Each process independently OPENs a device
        # session and then sits in readline with no firmware worker resident.
        for _ in range(holder_count):
            holders.append(subprocess.Popen(
                [CLI_BIN, "--usb"], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1))
        time.sleep(2.0)
        if any(p.poll() is not None for p in holders):
            return fail("an independent idle CLI exited unexpectedly")

        during_w = cli("w")
        during_count = session_count(during_w.stdout)
        if during_count is None or during_count < base_count + holder_count:
            return fail("w did not show every independent idle session", during_w.stdout)
        print(f"  w: {during_count} sessions ({holder_count} independently held open)")

        during_free_run = cli("free")
        during_free = free_bytes(during_free_run.stdout)
        if during_free is None:
            return fail("could not measure idle-session heap", during_free_run.stdout)
        idle_cost = base_free - during_free
        # Core session + WebUSB mailbox are about 80 B with heap metadata on a
        # 32-bit target. Keep generous allocator/scheduler variance while still
        # catching a fixed task, stack, or frame buffer accidentally added per
        # idle session.
        if idle_cost < 0 or idle_cost > holder_count * 256:
            return fail(f"idle session RAM cost is too high: {idle_cost} B for {holder_count}")
        print(f"  idle-session heap delta: {idle_cost} B total")

        # Several command workers can be active while the idle sessions remain.
        burst = [subprocess.Popen([CLI_BIN, "--usb", "-c", "whoami"],
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                  text=True) for _ in range(3)]
        for p in burst:
            out, err = p.communicate(timeout=20)
            if (p.returncode or "error:" in err.lower() or
                    len(device_lines(out)) != 1):
                return fail("concurrent whoami process failed", out + err)
        print("  concurrent whoami burst: OK")
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
                p.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
                p.communicate()

    # Keep a terminal-attached CLI idle beyond the firmware lease and verify
    # that readline heartbeats preserve its session.
    import pexpect
    idle = pexpect.spawn(CLI_BIN, ["--usb"], encoding="utf-8",
                         codec_errors="replace", timeout=30)
    try:
        idle.expect_exact("fantasi> ")
        time.sleep(20)
        idle.sendline("uptime")
        idle.expect_exact("fantasi> ", timeout=15)
        transcript = clean(idle.before or "")
        if "reconnected over USB" in transcript:
            return fail("idle WebUSB heartbeat did not preserve its session",
                        transcript)
        if not re.search(r"\d+m\s+\d+\.\d+s|\d+\.\d+s", transcript):
            return fail("idle WebUSB session did not answer after its lease window",
                        transcript)
        print("  idle WebUSB session survived beyond its lease: OK")
        idle.sendline("exit")
        idle.expect(pexpect.EOF, timeout=10)
    except (pexpect.TIMEOUT, pexpect.EOF):
        return fail("idle WebUSB lease regression timed out", idle.before or "")
    finally:
        if idle.isalive():
            idle.close(force=True)

    # A blocked stream owns only its session. A second process must still get a
    # short reply, and SIGINT must route a framed cancel to the stream owner.
    log = subprocess.Popen([CLI_BIN, "--usb", "-c", "log"],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           text=True)
    try:
        time.sleep(1.0)
        short = cli("whoami", timeout=15)
        if short.returncode:
            return fail("whoami blocked behind another process's log", short.stdout + short.stderr)
        listed = cli("w")
        count = session_count(listed.stdout)
        if count is None or count < base_count + 1:
            return fail("w did not show the blocked log session", listed.stdout)
        log.send_signal(signal.SIGINT)
        log.communicate(timeout=15)
        if log.returncode:
            return fail("cancellable log session did not exit cleanly")
        print("  blocked log + independent whoami + cancellation: OK")
    except subprocess.TimeoutExpired:
        return fail("blocked log session or independent command timed out")
    finally:
        if log.poll() is None:
            log.kill()
            log.communicate()

    # On targets with room for the stress, hold enough independent streams to
    # put the live task count above 16. The summary must match every row.
    # PM3 deliberately skips the worker fan-out: its much smaller heap is where
    # constant caller-space matters most, and its build/stack checks cover that
    # property without manufacturing an OOM just for this test.
    if platform != "proxmark3":
        streams = []
        stream_count = 10 if platform == "proxmark5" else 8
        try:
            for _ in range(stream_count):
                streams.append(subprocess.Popen(
                    [CLI_BIN, "--usb", "-c", "log"],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True))
            listed = None
            for _ in range(20):
                time.sleep(0.25)
                if any(p.poll() is not None for p in streams):
                    return fail("a ps-stress stream exited unexpectedly")
                listed = cli("w")
                if (session_count(listed.stdout) is not None and
                        session_count(listed.stdout) >= base_count + stream_count):
                    break
            if (listed is None or session_count(listed.stdout) is None or
                    session_count(listed.stdout) < base_count + stream_count):
                return fail("ps-stress sessions did not all become active",
                            "" if listed is None else listed.stdout)

            report = cli("ps", timeout=20)
            clean_report = clean(report.stdout)
            summary = re.search(r"^\s*(\d+) tasks\b", clean_report, re.MULTILINE)
            rows = re.findall(r"^\s*\d+\s+.+?\s+[RrBSD]\s+\d+\s+\d+ B\s*$",
                              clean_report, re.MULTILINE)
            if (report.returncode or not summary or int(summary.group(1)) <= 16 or
                    len(rows) != int(summary.group(1))):
                return fail("ps did not stream every task above its former cap",
                            report.stdout + report.stderr)
            print(f"  ps arbitrary-count stream: {summary.group(1)} tasks, "
                  "constant caller RAM")
        except subprocess.TimeoutExpired:
            return fail("ps arbitrary-count stress timed out")
        finally:
            for p in streams:
                if p.poll() is None:
                    p.send_signal(signal.SIGINT)
            for p in streams:
                try:
                    p.communicate(timeout=15)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.communicate()

    # Multi-frame requests are the path most likely to expose byte-stream
    # interleaving. Upload distinct payloads concurrently and verify both.
    with tempfile.TemporaryDirectory(prefix="fantasi-mux-") as td:
        form_content = "mux-upload-destination-0123456789\n" * 110
        form_paths = []
        for name in ("mux-cwd-default", "mux-cwd-dot"):
            local = os.path.join(td, name)
            with open(local, "w", encoding="utf-8") as f:
                f.write(form_content)
            form_paths.append(local)
        forms = cli(
            f"cd /ramfs;upload {form_paths[0]};"
            f"upload {form_paths[1]} .;"
            f"upload {form_paths[0]} /ramfs/mux-explicit",
            timeout=60)
        if forms.returncode or "failed" in forms.stderr.lower():
            return fail("cwd/default/dot upload destination failed",
                        forms.stdout + forms.stderr)
        for remote in ("/ramfs/mux-cwd-default", "/ramfs/mux-cwd-dot",
                       "/ramfs/mux-explicit"):
            got = cli(f"cat {remote}", timeout=30)
            if form_content.strip() not in clean(got.stdout):
                return fail(f"upload destination data mismatch for {remote}",
                            got.stdout + got.stderr)
            cli(f"rm {remote}")
        print("  cwd/default/dot upload destinations: OK")

        payloads = [
            # More than UPLOAD_WINDOW (six 480-byte chunks): a four-chunk test
            # cannot catch an unread ACK being discarded by the next WRITE.
            ("/ramfs/mux-a", ("mux-A-0123456789\n" * 220)),
            ("/ramfs/mux-b", ("mux-B-abcdefghij\n" * 220)),
        ]
        procs = []
        for i, (remote, content) in enumerate(payloads):
            local = os.path.join(td, f"payload-{i}.txt")
            with open(local, "w", encoding="utf-8") as f:
                f.write(content)
            procs.append(subprocess.Popen(
                [CLI_BIN, "--usb", "-c", f"upload {local} {remote}"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True))
        for p in procs:
            out, err = p.communicate(timeout=60)
            if p.returncode or "error:" in err.lower() or "failed" in err.lower():
                return fail("concurrent upload failed", out + err)
        for remote, content in payloads:
            got = cli(f"cat {remote}", timeout=30)
            if content.strip() not in clean(got.stdout):
                return fail(f"concurrent upload data mismatch for {remote}", got.stdout + got.stderr)
            cli(f"rm {remote}")
        print("  concurrent multi-frame uploads: OK")

    time.sleep(0.5)
    final_w = cli("w")
    final_count = session_count(final_w.stdout)
    if final_count is None or final_count != base_count:
        return fail("closed sessions remained in w", final_w.stdout)

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
