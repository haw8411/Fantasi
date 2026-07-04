#!/usr/bin/env python3
"""Unified Fantasi flash orchestrator.

Handles all three platforms with a consistent flow:
  1. Build resources (make storage) if the platform defines them
  2. Upload changed resources via MSC (LittleFS compare-before-write)
  3. Enter DFU / bootloader mode (send command over serial if needed)
  4. Flash firmware with the platform-specific tool

Usage:
    python3 tools/flash.py --platform <flipper|chameleon|proxmark3>

Exit codes:
    0  success
    1  fatal error (missing binary, missing tool, flash failure)
"""

import argparse
import glob
import os
import select
import subprocess
import sys
import tempfile
import time
import termios

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

USB_VID = "1209"
USB_PID = "0001"

PLATFORMS = {
    "flipper": {
        "id": "FZ",
        "bin": "build/flipper/fantasi-flipper.bin",
        "dfu_vid": "0483",
        "dfu_pid": "df11",
        "flash_tool": "dfu-util",
        "has_storage": True,
        "msc_mode": "composite",
    },
    # Kiisu is a Flipper-compatible STM32WB55 board - identical runtime
    # (1209:0001) and DFU (0483:df11) USB IDs, so it is told apart from a
    # Flipper only by the `device` command reporting "KIISU" vs "FZ".
    "kiisu": {
        "id": "KIISU",
        "bin": "build/kiisu/fantasi-kiisu.bin",
        "dfu_vid": "0483",
        "dfu_pid": "df11",
        "flash_tool": "dfu-util",
        "has_storage": True,
        "msc_mode": "composite",
    },
    "chameleon": {
        "id": "CU",
        "bin": "build/chameleon/fantasi-chameleon.hex",
        "dfu_vid": "1915",
        "dfu_pid": "521f",
        "flash_tool": "nrfutil",
        "has_storage": True,
        "msc_mode": "composite",
    },
    "proxmark3": {
        "id": "PM3",
        "bin": "build/proxmark3/fantasi-proxmark3.elf",
        "dfu_vid": "9ac4",
        "dfu_pid": "4b8f",
        "flash_tool": "pm3_flasher",
        "has_storage": True,
        "msc_mode": "switch",
    },
}

# ── USB device discovery ──────────────────────────────────────────────

def find_usb_device(vid, pid):
    """Find a USB device by VID:PID via sysfs. Returns sysfs path or None."""
    for usb_dev in glob.glob("/sys/bus/usb/devices/[0-9]*"):
        try:
            v = open(f"{usb_dev}/idVendor").read().strip()
            p = open(f"{usb_dev}/idProduct").read().strip()
        except OSError:
            continue
        if v == vid and p == pid:
            return usb_dev
    return None


def find_all_usb_devices(vid, pid):
    """All USB devices matching VID:PID (sysfs paths)."""
    out = []
    for usb_dev in glob.glob("/sys/bus/usb/devices/[0-9]*"):
        try:
            v = open(f"{usb_dev}/idVendor").read().strip()
            p = open(f"{usb_dev}/idProduct").read().strip()
        except OSError:
            continue
        if v == vid and p == pid:
            out.append(usb_dev)
    return out


def find_cdc_port(usb_dev=None):
    """Find the Fantasi CDC tty under a sysfs USB device node."""
    if usb_dev is None:
        usb_dev = find_usb_device(USB_VID, USB_PID)
    if usb_dev is None:
        return None
    for tty in glob.glob(f"{usb_dev}/*/tty/ttyACM*"):
        return "/dev/" + os.path.basename(tty)
    return None


def wait_for_usb(vid, pid, timeout=15):
    """Wait for a USB device to enumerate. Returns sysfs path or None."""
    for _ in range(timeout * 2):
        dev = find_usb_device(vid, pid)
        if dev:
            time.sleep(0.3)
            return dev
        time.sleep(0.5)
    return None


# ── Serial helpers ────────────────────────────────────────────────────

def send_serial_cmd(port, cmd):
    """Send a CLI command over CDC serial."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[3] = 0
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        os.write(fd, f"\r\n{cmd}\r\n".encode())
        time.sleep(0.3)
    finally:
        os.close(fd)


# Every platform's `device` id, longest first so a short id (FZ) can't shadow a
# longer one that would contain it. Derived from PLATFORMS so adding a board
# here is the only place its id needs to live.
DEVICE_IDS = sorted({cfg["id"] for cfg in PLATFORMS.values()}, key=len, reverse=True)


def query_device_id(port):
    """Ask a running Fantasi over CDC what it is (the `device` command).
    Returns the matching platform id ('FZ' / 'KIISU' / 'CU' / 'PM3'), or None if
    it doesn't answer with a known one."""
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[3] = 0
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        while select.select([fd], [], [], 0.05)[0]:   # flush stale input
            try:
                if not os.read(fd, 256):
                    break
            except OSError:
                break
        os.write(fd, b"\r\ndevice\r\n")
        buf = b""
        end = time.time() + 1.5
        while time.time() < end:
            if select.select([fd], [], [], 0.2)[0]:
                try:
                    d = os.read(fd, 256)
                except OSError:
                    break
                if d:
                    buf += d
            if any(i.encode() in buf for i in DEVICE_IDS):
                break
        for i in DEVICE_IDS:
            if i.encode() in buf:
                return i
        return None
    finally:
        os.close(fd)


def select_running_device(cfg):
    """Find the single RUNNING Fantasi whose `device` id matches cfg['id'].
    Returns (usb_dev, cdc_port); (None, None) if none is running. Exits if more
    than one matches - refusing to flash so we can't target the wrong board."""
    matches = []
    for usb_dev in find_all_usb_devices(USB_VID, USB_PID):
        port = find_cdc_port(usb_dev)
        if port and query_device_id(port) == cfg["id"]:
            matches.append((usb_dev, port))
    if len(matches) > 1:
        ports = ", ".join(p for _, p in matches)
        sys.exit(f"error: {len(matches)} {cfg['id']} devices connected ({ports}) "
                 f"- refusing to flash. Connect only the target device.")
    return matches[0] if matches else (None, None)


# FZ vs Kiisu in DFU
# The Flipper OTP (0x1FFF7000) carries an 8-byte device-name field at +24; the
# Kiisu's factory provisioning space-pads unused bytes (0x20) while genuine
# Flippers NUL-pad (0x00). Everything else DFU exposes - option bytes,
# USB descriptors, the rest of the OTP - is identical or per-unit noise.
#
# We read the LAST (8th) byte of that field and decide strictly:
#   0x20 -> Kiisu,  0x00 -> Flipper,  anything else -> can't tell, ask the user.
# The "anything else" catches an 8-character name (byte 8 is a real character,
# so there's no padding to read) - for those we refuse to guess and prompt.
OTP_NAME_LAST_ADDR = 0x1FFF7000 + 24 + 7   # 8th byte of the 8-byte name field


def dfu_probe_name_pad():
    """Read the 8th byte of the OTP name field from a board in the shared
    FZ/Kiisu DFU mode. Returns that byte (int), or None if it couldn't be read."""
    tmp = tempfile.mkdtemp(prefix="fantasi-otp-")
    path = os.path.join(tmp, "otp.bin")   # must not pre-exist: dfu-util won't overwrite
    try:
        subprocess.run(
            ["dfu-util", "-d", "0483:df11", "-a", "2",
             "-s", "0x%X:1" % OTP_NAME_LAST_ADDR, "-U", path],
            capture_output=True, timeout=30)
        with open(path, "rb") as f:
            b = f.read(1)
    except (OSError, subprocess.SubprocessError):
        return None
    finally:
        try:
            os.remove(path)
        except OSError:
            pass
        try:
            os.rmdir(tmp)
        except OSError:
            pass
    return b[0] if len(b) == 1 else None


def prompt_platform(candidates):
    """Ask the user which platform to flash, among `candidates`. Exits if there
    is no terminal to prompt on."""
    opts = sorted(candidates)
    if not sys.stdin.isatty():
        sys.exit("error: can't tell " + " from ".join(opts) + " apart in DFU "
                 "(OTP name field is not padding) and no terminal to prompt.\n"
                 "  Re-run with an explicit target, e.g. make PLATFORM=" + opts[0]
                 + " flash")
    while True:
        # Prompt on stderr so --detect's stdout stays a bare platform name
        # (the Makefile captures stdout as `plat`).
        print("DFU device is ambiguous - flash which platform? ["
              + "/".join(opts) + "]: ", end="", file=sys.stderr, flush=True)
        ans = input().strip().lower()
        if ans in opts:
            return ans
        print("  please type one of: " + ", ".join(opts), file=sys.stderr)


def detect_platform():
    """Auto-detect the single connected platform. A running Fantasi is
    identified by its `device` id (authoritative - the firmware knows its own
    build). A board in DFU is identified by its DFU USB id; the Flipper and Kiisu
    share one (0483:df11) so they're disambiguated by the OTP name-padding probe
    (dfu_probe_is_kiisu). Exits on zero / conflicting / unreadable detection."""
    id_to_plat = {cfg["id"]: name for name, cfg in PLATFORMS.items()}

    running = set()
    for usb_dev in find_all_usb_devices(USB_VID, USB_PID):   # running Fantasi
        port = find_cdc_port(usb_dev)
        if port:
            did = query_device_id(port)
            if did in id_to_plat:
                running.add(id_to_plat[did])

    # DFU boards grouped by (vid,pid): a shared id names a SET of candidates,
    # not one platform. FZ and Kiisu both live under 0483:df11.
    dfu_groups = {}
    for name, cfg in PLATFORMS.items():
        key = (cfg["dfu_vid"], cfg["dfu_pid"])
        if find_all_usb_devices(*key):
            dfu_groups.setdefault(key, set()).add(name)

    found = set(running)
    for names in dfu_groups.values():
        if names & running:
            continue                    # a running query already resolved it
        if len(names) == 1:
            found |= names              # uniquely-identified DFU board (CU, PM3)
        elif names == {"flipper", "kiisu"}:
            # Shared 0483:df11 - decide on the 8th OTP name byte (see above):
            # 0x20 -> Kiisu, 0x00 -> Flipper, anything else -> ask.
            # Log to stderr: --detect's stdout must be only the platform name.
            pad = dfu_probe_name_pad()
            if pad == 0x20:
                print("DFU device: OTP name pad byte is 0x20 -> Kiisu", file=sys.stderr)
                found.add("kiisu")
            elif pad == 0x00:
                print("DFU device: OTP name pad byte is 0x00 -> Flipper", file=sys.stderr)
                found.add("flipper")
            else:
                where = "unreadable" if pad is None else ("0x%02X" % pad)
                print("DFU device: OTP name pad byte is %s - can't tell "
                      "Flipper from Kiisu" % where, file=sys.stderr)
                found.add(prompt_platform(names))
        else:                           # some other future shared id
            sys.exit("error: a device is in DFU mode, but these share a DFU id "
                     "and can't be told apart there: " + ", ".join(sorted(names))
                     + ".\n  Re-run with an explicit target, e.g. make PLATFORM="
                     + sorted(names)[0] + " flash")

    if len(found) > 1:
        sys.exit("error: multiple platforms detected (" + ", ".join(sorted(found))
                 + ") - specify one, e.g. make PLATFORM=" + sorted(found)[0]
                 + " flash")
    if not found:
        sys.exit("error: no Fantasi device detected - connect one "
                 "(running or in DFU)")
    return next(iter(found))


# ── Resource upload (via the host CLI) ───────────────────────────────
#
# The host CLI (build/cli/fantasi) owns the device's storage path: it switches
# the device into MSC mode if needed, OS-mounts the synthetic "Fantasi" FAT, and
# does file I/O with stdio - then returns the device to CDC on exit. The flasher
# drives it instead of touching the block device directly, so there is no
# on-disk-format coupling (no littlefs-python) and one tool owns the MSC dance.

import re as _re

CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")


def ensure_cli():
    """Return the host CLI binary path, building it on demand. None on failure."""
    if os.path.isfile(CLI_BIN):
        return CLI_BIN
    print("Building host CLI...")
    r = subprocess.run(["make", "-C", os.path.join(REPO_ROOT, "cli")])
    return CLI_BIN if r.returncode == 0 and os.path.isfile(CLI_BIN) else None


def _cli(cli_bin, cdc_port, script, timeout=180):
    r = subprocess.run([cli_bin, cdc_port], input=script,
                       capture_output=True, text=True, timeout=timeout)
    return _re.sub(r'\033\[[0-9;]*m', '', r.stdout)


def upload_resources(cli_bin, cdc_port, resources):
    """Upload changed resource files via the CLI: crc32-compare, then upload only
    what differs (avoids needless flash writes). Returns True if anything changed."""
    import zlib

    # 1. Ask the device for the CRC32 of each resource.
    script = "".join(f"crc32 {remote}\n" for _, remote in resources) + "exit\n"
    out = _cli(cli_bin, cdc_port, script)
    have = {}
    for line in out.splitlines():
        m = _re.match(r'^([0-9a-f]{8})\s+\d+\s+(\S+)', line)
        if m:
            have[m.group(2)] = int(m.group(1), 16)

    # 2. Decide which differ.
    todo = []
    for local, remote in resources:
        with open(local, "rb") as f:
            want = zlib.crc32(f.read()) & 0xffffffff
        if have.get(remote) == want:
            print(f"  {remote}: unchanged")
        else:
            todo.append((local, remote))

    if not todo:
        return False

    # 3. Upload the changed ones (re-resolve the port after the MSC cycle above).
    time.sleep(1)
    cdc_port = find_cdc_port() or cdc_port
    script = "".join(f"upload {local} {remote}\n" for local, remote in todo) + "exit\n"
    out = _cli(cli_bin, cdc_port, script)
    for local, remote in todo:
        print(f"  {remote}: {'uploaded' if remote in out else 'upload FAILED'}")
    return True


# ── Resource collection ───────────────────────────────────────────────

def collect_resources(platform):
    """Return list of (local_path, remote_path) for a platform, or []."""
    resources = []
    # Flipper and the pin-identical Kiisu both ship the LCD splash bitmap.
    if platform in ("flipper", "kiisu"):
        splash = os.path.join(REPO_ROOT, f"build/{platform}/splash.bin")
        if os.path.isfile(splash):
            resources.append((splash, "/splash.bin"))
    return resources


# ── Make storage ──────────────────────────────────────────────────────

def build_storage(platform):
    """Run `make PLATFORM=<x> storage` if the platform has a storage target."""
    r = subprocess.run(
        ["make", f"PLATFORM={platform}", "storage"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    if r.returncode != 0 and "No rule to make target" not in r.stderr:
        print(r.stderr, file=sys.stderr)
        return False
    return True


# ── udev rules ────────────────────────────────────────────────────────

def ensure_udev_rules():
    src = os.path.join(REPO_ROOT, "tools/50-fantasi.rules")
    dst = "/etc/udev/rules.d/50-fantasi.rules"
    if not os.path.isfile(src) or os.path.isfile(dst):
        return
    print("Installing udev rules (one-time, requires sudo)...")
    subprocess.run(["sudo", "cp", src, dst], check=True)
    subprocess.run(["sudo", "udevadm", "control", "--reload-rules"], check=True)
    subprocess.run(["sudo", "udevadm", "trigger"], check=True)
    time.sleep(1)


# ── Platform-specific flash commands ─────────────────────────────────

def flash_flipper(bin_path):
    # dfu-util with :leave often returns exit 74 ("Error during download
    # get_status") because the device resets before the host reads the
    # final status.  The flash itself succeeds - treat 74 as success.
    r = subprocess.run(
        ["dfu-util", "-a", "0", "-d", "0483:df11",
         "-s", "0x08000000:leave", "-D", bin_path],
    )
    if r.returncode not in (0, 74):
        sys.exit(r.returncode)

    # Confirm the app re-enumerated instead of trusting the leave. If the device
    # is still in DFU, fail loudly - a silent "Flash complete" on a stranded
    # device breaks every step that follows.
    if not wait_for_usb(USB_VID, USB_PID, timeout=15):
        print("error: device did not re-enumerate after flashing - still in DFU?",
              file=sys.stderr)
        sys.exit(1)


def flash_chameleon(hex_path):
    build_dir = os.path.dirname(hex_path)
    zip_path = os.path.join(build_dir, "fantasi-chameleon-dfu.zip")
    key_path = os.path.join(REPO_ROOT, "platforms/chameleon/dfu_key.pem")

    if not os.path.isfile(key_path):
        print(f"error: DFU signing key missing at {key_path}", file=sys.stderr)
        sys.exit(1)

    print("Packaging DFU zip...")
    subprocess.run([
        "nrfutil", "pkg", "generate",
        "--hw-version", "0",
        "--sd-req", "0x00",
        "--application-version", "1",
        "--application", hex_path,
        "--key-file", key_path,
        zip_path,
    ], check=True)

    dfu_dev = find_usb_device("1915", "521f")
    if not dfu_dev:
        print("error: CU DFU device not found", file=sys.stderr)
        sys.exit(1)

    dfu_port = find_cdc_port(dfu_dev)
    if not dfu_port:
        for tty in sorted(glob.glob("/dev/ttyACM*")):
            dfu_port = tty
            break
    if not dfu_port:
        print("error: no serial port found for nrfutil DFU", file=sys.stderr)
        sys.exit(1)

    print(f"Flashing via nrfutil on {dfu_port}...", flush=True)
    subprocess.run([
        "nrfutil", "dfu", "usb-serial",
        "-pkg", zip_path,
        "-p", dfu_port,
    ], check=True)


def flash_proxmark3(elf_path):
    flasher = os.path.join(REPO_ROOT, "tools/pm3_flasher.py")
    dfu_dev = find_usb_device("9ac4", "4b8f")
    if not dfu_dev:
        print("error: PM3 bootloader device not found", file=sys.stderr)
        sys.exit(1)

    port = find_cdc_port(dfu_dev)
    if not port:
        for tty in sorted(glob.glob("/dev/ttyACM*")):
            port = tty
            break
    if not port:
        print("error: no serial port found for PM3 flasher", file=sys.stderr)
        sys.exit(1)

    print(f"Flashing via pm3_flasher on {port}...", flush=True)
    subprocess.run(["python3", flasher, port, elf_path], check=True)


# ── Main flow ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Fantasi unified flasher")
    parser.add_argument("--platform", choices=PLATFORMS.keys(),
                        help="target platform; auto-detected if omitted")
    parser.add_argument("--detect", action="store_true",
                        help="print the auto-detected platform and exit "
                             "(errors if zero or multiple are connected)")
    args = parser.parse_args()

    if args.detect:
        print(detect_platform())
        return 0

    # No --platform: auto-detect the single connected board (errors on ambiguity).
    plat = args.platform or detect_platform()
    cfg = PLATFORMS[plat]
    bin_path = os.path.join(REPO_ROOT, cfg["bin"])

    if not os.path.isfile(bin_path):
        print(f"error: {bin_path} not found - run 'make PLATFORM={plat}' first",
              file=sys.stderr)
        sys.exit(1)

    ensure_udev_rules()

    # Step 1: Build resources
    if cfg["has_storage"]:
        print("Building resources...")
        build_storage(plat)

    resources = collect_resources(plat)

    # Step 2: Sync resources via the host CLI if the device is running Fantasi.
    # Select ONLY the device matching the requested platform (by its `device`
    # id), so a second connected board can't be flashed by mistake. The CLI
    # owns the MSC switch + FAT mount and returns the device to CDC on exit, so
    # the flasher only has to invoke it and re-resolve the port afterwards.
    # (/apps is created by the firmware on boot, so it needs no host step.)
    fantasi_dev, cdc_port = select_running_device(cfg)

    if resources and cdc_port:
        cli_bin = ensure_cli()
        if cli_bin:
            print("Syncing resources...")
            upload_resources(cli_bin, cdc_port, resources)
            time.sleep(2)
            fantasi_dev, cdc_port = select_running_device(cfg)
        else:
            print("warning: host CLI unavailable, skipping resource sync")
    elif resources and not cdc_port:
        print("warning: device not running Fantasi, skipping resource sync")

    # Step 3: Enter DFU / bootloader mode. The DFU VID:PID is unique per
    # platform, so a DFU device already present unambiguously identifies the
    # target - but bail if somehow more than one of them is attached.
    dfu_devs = find_all_usb_devices(cfg["dfu_vid"], cfg["dfu_pid"])
    if len(dfu_devs) > 1:
        sys.exit(f"error: {len(dfu_devs)} {plat} DFU devices "
                 f"({cfg['dfu_vid']}:{cfg['dfu_pid']}) connected - refusing to "
                 f"flash. Connect only the target device.")
    dfu_dev = dfu_devs[0] if dfu_devs else None

    # Nothing matched: neither a running Fantasi of this platform nor its DFU
    # device is present. Refuse rather than fall through to a wrong target.
    if not dfu_dev and not cdc_port:
        sys.exit(f"error: no {plat} device ({cfg['id']}) found - connect it "
                 f"(running Fantasi or already in DFU) and retry.")

    # The `dfu` CDC command is occasionally missed when sent immediately after a
    # large MSC write (the device USB stack is still draining), so retry a few
    # times rather than failing on a single 15 s wait.
    if not dfu_dev and cdc_port:
        for attempt in range(3):
            print(f"Sending DFU command via {cdc_port} (attempt {attempt + 1})...")
            send_serial_cmd(cdc_port, "dfu")
            print(f"Waiting for DFU device ({cfg['dfu_vid']}:{cfg['dfu_pid']})...")
            dfu_dev = wait_for_usb(cfg["dfu_vid"], cfg["dfu_pid"])
            if dfu_dev:
                break

    if not dfu_dev:
        print()
        print("Device not detected. Enter DFU/bootloader mode manually:")
        if plat in ("flipper", "kiisu"):
            print("  Hold OK + BACK for 30 seconds until the screen goes blank.")
        elif plat == "chameleon":
            print("  Unplug USB, hold the user button, plug USB back in.")
        elif plat == "proxmark3":
            print("  Hold the button on the Proxmark3 while plugging in USB.")
        print()
        # Only prompt when a human is present; under a non-interactive runner
        # (e.g. the test harness) a missing stdin must surface as a clean error,
        # not an EOFError traceback.
        if sys.stdin.isatty():
            input("Press enter once the device is in DFU mode... ")
            dfu_dev = find_usb_device(cfg["dfu_vid"], cfg["dfu_pid"])
        if not dfu_dev:
            print(f"error: DFU device {cfg['dfu_vid']}:{cfg['dfu_pid']} still not found",
                  file=sys.stderr)
            sys.exit(1)

    # Step 4: Flash
    print(f"Flashing {os.path.basename(bin_path)}...", flush=True)

    if plat in ("flipper", "kiisu"):   # Kiisu shares the STM32WB DFU path (0483:df11)
        flash_flipper(bin_path)
    elif plat == "chameleon":
        flash_chameleon(bin_path)
    elif plat == "proxmark3":
        flash_proxmark3(bin_path)
    else:
        sys.exit(f"error: no flash routine for platform '{plat}'")

    print()
    print("Flash complete.")


if __name__ == "__main__":
    main()
