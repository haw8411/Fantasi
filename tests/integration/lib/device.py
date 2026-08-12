"""Shared device discovery and interaction helpers for hardware tests."""

import glob
import os
import time

USB_VID = "1209"
USB_PID = "0001"

PLATFORMS = {
    "flipper": {
        "bin": "build/flipper/fantasi-flipper.bin",
        "dfu_vid": "0483",
        "dfu_pid": "df11",
        "msc_mode": "composite",
        "app_arch": "cm4",          # Cortex-M4 app ELF variant
    },
    "kiisu": {                      # Flipper-compatible STM32WB55 board (kiisu.io v4)
        "bin": "build/kiisu/fantasi-kiisu.bin",
        "dfu_vid": "0483",          # ST system bootloader (same as Flipper)
        "dfu_pid": "df11",
        "msc_mode": "composite",
        "app_arch": "cm4",
    },
    "chameleon": {
        "bin": "build/chameleon/fantasi-chameleon.hex",
        "dfu_vid": "1915",
        "dfu_pid": "521f",
        "msc_mode": "composite",
        "app_arch": "cm4",
    },
    "proxmark3": {
        "bin": "build/proxmark3/fantasi-proxmark3.elf",
        "dfu_vid": "9ac4",
        "dfu_pid": "4b8f",
        "msc_mode": "switch",
        "app_arch": "arm7",         # ARM7TDMI app ELF variant
    },
    "proxmark5": {
        "bin": "build/proxmark5/fantasi-proxmark5.bin",
        "dfu_vid": "2e3c",          # AT32 ROM DFU
        "dfu_pid": "df11",
        "msc_mode": "composite",
        "app_arch": "cm4",
        # The AT32 ROM DFU can't hand control back to the app over USB the way the
        # Flipper's STM32 ROM does: `:leave` is a full system reset that wipes the
        # GPIOB-refresh DMA holding the PB0 power-latch, so the board powers off in
        # the ROM's post-reset boot window and needs a manual BOOT0-low power-cycle.
        # Tests that reflash and expect the app to re-enumerate unattended must skip.
        "no_auto_dfu_return": True,
    },
}


def build_app(repo_root, name, platform):
    """Build apps/<name> and return the path to the ELF variant for `platform`
    (build/apps/<name>.<arch>.elf), or None if the toolchain or source is
    unavailable. `make app` produces every variant; we return the matching one."""
    import subprocess
    arch = PLATFORMS[platform]["app_arch"]
    elf = os.path.join(repo_root, "build", "apps", f"{name}.{arch}.elf")
    try:
        subprocess.run(["make", "app", f"APP={name}"], cwd=repo_root,
                       capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.SubprocessError):
        pass
    return elf if os.path.isfile(elf) else None


def find_usb_device(vid, pid):
    for usb_dev in glob.glob("/sys/bus/usb/devices/[0-9]*"):
        try:
            v = open(f"{usb_dev}/idVendor").read().strip()
            p = open(f"{usb_dev}/idProduct").read().strip()
        except OSError:
            continue
        if v == vid and p == pid:
            return usb_dev
    return None


# Handed to the fantasi CLI when the device is present but exposes no CDC tty -
# i.e. a switch-mode device (PM3) already in WebUSB mode. It is deliberately a
# non-existent /dev/tty* path: the CLI finds the serial port gone and falls back
# to the WebUSB vendor pipe (connect_webusb_direct in cli/main.c). The first CLI
# invocation of a suite auto-upgrades the PM3 to WebUSB and it stays there, so
# this is what lets the per-test invocations that follow (upload, then cat, then
# rm - each a separate CLI run) keep reaching the device.
CLI_WEBUSB_SENTINEL = "/dev/ttyFANTASI_WEBUSB"


def find_cdc_port(usb_dev=None):
    if usb_dev is None:
        # A switch-mode device (PM3) briefly drops off /sys while it re-enumerates
        # between CDC and WebUSB; wait for it to reappear rather than racing the
        # next test's discovery. Composite devices are always present, so this is
        # a no-op there.
        usb_dev = find_usb_device(USB_VID, USB_PID) or wait_for_usb(USB_VID, USB_PID, timeout=15)
    if usb_dev is None:
        return None
    # The USB device can show up in /sys a beat before its CDC tty node, so poll
    # briefly - otherwise a device re-enumerating into CDC mode is mistaken for
    # WebUSB-only and routed to the vendor pipe (which re-switches it, churning).
    for _ in range(12):
        for tty in glob.glob(f"{usb_dev}/*/tty/ttyACM*"):
            return "/dev/" + os.path.basename(tty)
        time.sleep(0.25)
    # Device present but no CDC tty: a switch-mode device already in WebUSB mode.
    # Hand the CLI a non-existent serial path so it uses the vendor pipe directly.
    return CLI_WEBUSB_SENTINEL


def wait_for_usb(vid, pid, timeout=30):
    for _ in range(timeout * 2):
        dev = find_usb_device(vid, pid)
        if dev:
            time.sleep(0.5)
            return dev
        time.sleep(0.5)
    return None


def ensure_cdc(timeout=25):
    """Return a switch-mode device (PM3) to serial (CDC) if it's currently in
    WebUSB mode - i.e. vendor-only with no CDC port, e.g. left there by a prior
    test whose CLI auto-upgraded it. Sends the `cdc` protobuf frame over the
    vendor bulk pipe (retrying, since a lone OUT packet can be dropped) until the
    CDC port reappears. No-op returning True when a CDC port already exists, so
    it is safe to call unconditionally (composite FZ/CU always have CDC).
    Returns True once CDC is available.
    """
    # NB: find_cdc_port() returns CLI_WEBUSB_SENTINEL (truthy) when the device is
    # in WebUSB, so check for a REAL /dev/ttyACM* - otherwise this would no-op
    # exactly when a switch back to CDC is needed.
    def on_cdc():
        return find_cdc_port() not in (None, CLI_WEBUSB_SENTINEL)
    if on_cdc():
        return True
    try:
        import usb.core
        import usb.util
        import struct
    except ImportError:
        return on_cdc()

    # CliRequest{ id=1, command="cdc" }, 2-byte LE length prefix (same wire
    # framing as the BLE/WebUSB protobuf transport).
    body = bytes([0x08, 0x01, 0x12, 0x03]) + b"cdc"
    frame = struct.pack("<H", len(body)) + body

    deadline = time.time() + timeout
    while time.time() < deadline:
        if on_cdc():
            return True
        dev = usb.core.find(idVendor=int(USB_VID, 16), idProduct=int(USB_PID, 16))
        if dev is not None:
            try:
                itf = next((i for i in dev.get_active_configuration()
                            if i.bInterfaceClass == 0xFF), None)
                if itf is not None:
                    usb.util.claim_interface(dev, itf.bInterfaceNumber)
                    epo = usb.util.find_descriptor(
                        itf, custom_match=lambda e:
                        usb.util.endpoint_direction(e.bEndpointAddress)
                        == usb.util.ENDPOINT_OUT)
                    epi = usb.util.find_descriptor(
                        itf, custom_match=lambda e:
                        usb.util.endpoint_direction(e.bEndpointAddress)
                        == usb.util.ENDPOINT_IN)
                    epo.write(frame)
                    try:
                        epi.read(64, timeout=800)   # drain reply / re-arm OUT bank
                    except Exception:
                        pass                        # disconnect mid-switch is expected
                    usb.util.dispose_resources(dev)
            except Exception:
                pass
        time.sleep(1)
    return on_cdc()


def webusb_send(cmd, read_secs=1.5):
    """Send one CLI command over the WebUSB vendor pipe (pyusb, framed protobuf)
    and return the raw response bytes read for read_secs. This is a second channel
    that does NOT touch CDC, so it can drive a device while a serial session runs
    an app - used to test cross-channel `kill`. Returns b'' if pyusb or the vendor
    interface is unavailable. cmd must be < 128 bytes.
    """
    try:
        import usb.core
        import usb.util
        import struct
    except ImportError:
        return b''
    d = usb.core.find(idVendor=int(USB_VID, 16), idProduct=int(USB_PID, 16))
    if d is None:
        return b''
    itf = next((i for i in d.get_active_configuration()
                if i.bInterfaceClass == 0xFF), None)
    if itf is None:
        return b''
    try:
        usb.util.claim_interface(d, itf.bInterfaceNumber)
        epo = usb.util.find_descriptor(itf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
        epi = usb.util.find_descriptor(itf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
        # CliRequest{ id=1, command=cmd } + 2-byte LE length prefix.
        body = bytes([0x08, 0x01, 0x12, len(cmd)]) + cmd.encode()
        epo.write(struct.pack("<H", len(body)) + body)
        data = b""
        t0 = time.time()
        while time.time() - t0 < read_secs:
            try:
                data += bytes(epi.read(256, timeout=200))
            except Exception:
                pass
        return data
    except Exception:
        return b''
    finally:
        try:
            usb.util.dispose_resources(d)
        except Exception:
            pass


def send_serial_cmd(port, cmd, timeout=2):
    """Send a CLI command and return the output lines (prompt/echo stripped)."""
    import serial
    with serial.Serial(port, 115200, timeout=timeout) as ser:
        ser.write(b"\r\n")
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.write(f"{cmd}\r\n".encode())
        time.sleep(0.3)
        try:
            raw = ser.read(ser.in_waiting or 256).decode(errors="replace")
        except OSError:
            return ""
    lines = [
        l for l in raw.splitlines()
        if l and not l.startswith("fantasi>") and l != cmd
        and "CLI ready" not in l
    ]
    return "\n".join(lines)


