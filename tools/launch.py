#!/usr/bin/env python3
"""Build, upload, and launch a Fantasi app on the connected device.

Steps:
  1. Build the app for the connected board's architecture (make app APP=<name>).
  2. Upload the matching ELF variant to /ramfs (a separate, non-streaming CLI
     session - RAM-backed files survive between sessions).
  3. Open an *interactive* `launch` session over a PTY so the app's output
     streams to your terminal and Ctrl-C is forwarded to the device as 0x03,
     which the firmware's launch loop treats as "stop the app" (a clean kill
     that frees the image + task), then drops you back at the prompt.

Usage:
    python3 tools/launch.py <name> [--platform <p>] [--dest <device-path>]
    (or: make launch APP=<name>)
"""

import argparse
import os
import pty
import select
import subprocess
import sys
import termios
import time
import tty

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from flash import (   # imported after sys.path is set above
    PLATFORMS, USB_VID, USB_PID,
    detect_platform, find_usb_device, find_cdc_port,
)

CLI_BIN = os.path.join(REPO_ROOT, "build/cli/fantasi")

# App ELF variant per target core (matches apps/Makefile's outputs).
# All Cortex-M targets share the cm4 app variant; only the ARM7TDMI PM3 differs.
APP_ARCH = {"flipper": "cm4", "kiisu": "cm4", "chameleon": "cm4",
            "proxmark3": "arm7", "proxmark5": "cm4"}


def wait_cdc(timeout=15):
    """Re-find the CDC port, tolerating a re-enumeration (PM3 switch-mode
    ejects the MSC LUN after the upload, so the tty briefly disappears)."""
    end = time.time() + timeout
    while time.time() < end:
        usb = find_usb_device(USB_VID, USB_PID)
        if usb:
            port = find_cdc_port(usb)
            if port:
                return port
        time.sleep(0.5)
    return None


def interactive_launch(cli_args, command):
    """Run the host CLI (`cli_args` selects the transport: [port] for USB, or
    ["--ble", ...] for BLE) attached to a PTY, inject `command` once its prompt
    appears, then bridge the user's terminal to it until the CLI exits.

    The PTY (not a pipe) is what makes Ctrl-C work: with the terminal in raw
    mode the keystroke arrives as a 0x03 byte that we forward to the CLI, whose
    launch stream loop stops the app on 0x03 - rather than SIGINT killing the
    host process and orphaning the running app on the device. It also lets a
    BLE pairing passkey prompt be answered interactively."""
    pid, fd = pty.fork()
    if pid == 0:                                  # child: become the CLI
        os.execvp(CLI_BIN, [CLI_BIN, *cli_args])
        os._exit(127)                             # unreachable on success

    interactive = sys.stdin.isatty()
    restore = None
    if interactive:
        try:
            restore = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except (termios.error, ValueError):
            interactive = False

    watch = [fd, sys.stdin] if interactive else [fd]
    prompts = 0          # 1st prompt -> inject launch; its return -> 2nd prompt
    pending = b""
    try:
        while True:
            try:
                rlist, _, _ = select.select(watch, [], [])
            except (OSError, select.error):
                break
            if fd in rlist:
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    data = b""
                if not data:
                    break                         # CLI exited
                os.write(sys.stdout.fileno(), data)
                pending += data
                while b"fantasi>" in pending:
                    pending = pending.split(b"fantasi>", 1)[1]
                    prompts += 1
                    if prompts == 1:              # banner shown - run the app
                        os.write(fd, command.encode())
                    elif not interactive:         # app returned - leave (batch)
                        os.write(fd, b"exit\n")
            if interactive and sys.stdin in rlist:
                data = os.read(sys.stdin.fileno(), 4096)
                if not data:                       # local EOF: ask the CLI to quit
                    os.write(fd, b"\nexit\n")
                    watch = [fd]
                    continue
                os.write(fd, data)                # forwards Ctrl-C (0x03) too
    finally:
        if restore is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, restore)
        try:
            os.close(fd)
        except OSError:
            pass
        _, status = os.waitpid(pid, 0)
    return os.waitstatus_to_exitcode(status)


def main():
    ap = argparse.ArgumentParser(description="Launch a Fantasi app on the device")
    ap.add_argument("name", help="app name under apps/<name>/ (e.g. hello)")
    ap.add_argument("--platform", choices=PLATFORMS.keys(),
                    help="target platform; auto-detected if omitted")
    ap.add_argument("--dest", help="device path to upload to (default /ramfs/<name>)")
    ap.add_argument("--ble", action="store_true",
                    help="reach the device over BLE instead of USB")
    ap.add_argument("--ble-addr",
                    help="BLE address of the device (implies --ble; else auto-discover)")
    args = ap.parse_args()
    if args.ble_addr:
        args.ble = True

    if not os.path.isfile(os.path.join(REPO_ROOT, "apps", args.name, args.name + ".c")):
        sys.exit(f"error: apps/{args.name}/{args.name}.c not found")

    # Pick the app architecture. Only the composite Cortex-M targets (Flipper,
    # Chameleon) have BLE, so a BLE launch is always the cm4 variant; over USB we
    # auto-detect the connected board.
    if args.ble:
        if args.platform == "proxmark3":
            sys.exit("error: the Proxmark3 has no BLE")
        arch = APP_ARCH[args.platform] if args.platform else "cm4"
        label = args.platform or "ble"
    else:
        label = args.platform or detect_platform()
        arch = APP_ARCH[label]
    dest = args.dest or f"/ramfs/{args.name}"

    # The host CLI must exist (used for both upload and launch).
    if not os.path.isfile(CLI_BIN):
        subprocess.run(["make", "cli"], cwd=REPO_ROOT, check=True)

    # 1. Build the app (produces build/apps/<name>.{cm4,arm7}.elf).
    subprocess.run(["make", "app", f"APP={args.name}"], cwd=REPO_ROOT, check=True)
    elf = os.path.join(REPO_ROOT, f"build/apps/{args.name}.{arch}.elf")
    if not os.path.isfile(elf):
        sys.exit(f"error: {elf} was not built")

    # 2. Select the transport. USB needs the CDC port; BLE lets the CLI discover
    # (and pair, if needed) the device itself - optionally pinned by --ble-addr.
    if args.ble:
        transport = ["--ble"] + ([f"--ble-addr={args.ble_addr}"] if args.ble_addr else [])
    else:
        port = wait_cdc()
        if not port:
            sys.exit("error: no Fantasi device found (is it connected?)")
        transport = [port]

    # 3. Upload the app.
    rel = os.path.relpath(elf, REPO_ROOT)
    print(f"Uploading {rel} -> {dest}  ({label})", flush=True)
    up = subprocess.run([CLI_BIN, *transport], input=f"upload {elf} {dest}\nexit\n",
                        capture_output=True, text=True, timeout=120)
    if dest not in up.stdout and args.name not in up.stdout:
        hint = ""
        if args.ble and "passkey" in (up.stdout + up.stderr).lower():
            hint = ("\nhint: the device isn't bonded yet - pair it once interactively "
                    "first (e.g. `build/cli/fantasi --ble`), then retry.")
        sys.exit(f"error: upload failed{hint}\n{up.stdout}\n{up.stderr}")

    # 4. Interactive launch over the same transport (USB: re-find the port in case
    # the upload re-enumerated it; BLE: reuse the same args, CLI reconnects).
    if not args.ble:
        transport = [wait_cdc() or transport[0]]
    print(f"Launching {dest} - press Ctrl-C to stop the app\n", flush=True)
    return interactive_launch(transport, f"launch {dest}\n")


if __name__ == "__main__":
    sys.exit(main())
