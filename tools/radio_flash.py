#!/usr/bin/env python3
"""Flash an STM32WB wireless stack binary via the ROM DFU bootloader.

Implements the same protocol as qFlipper: FUS commands as block-0 DFU_DNLOAD,
FUS state reads via DFU_UPLOAD from 0xFFFF0054.

Usage:
    1. Put the Flipper in DFU mode:  fantasi> dfu
    2. Run:  python3 tools/radio_flash.py <stack.bin>
"""

import sys
import time
import struct
import math
import usb.core
import usb.util

VID, PID, INTF = 0x0483, 0xDF11, 0
FLASH_PAGE = 4096

# DFU requests
DFU_DNLOAD, DFU_UPLOAD, DFU_GETSTATUS, DFU_CLRSTATUS, DFU_ABORT = 1, 2, 3, 4, 6
REQ_OUT = 0x21  # host-to-device, class, interface
REQ_IN  = 0xA1  # device-to-host, class, interface

# DfuSe special commands
DFUSE_SET_ADDRESS = 0x21
DFUSE_ERASE_PAGE  = 0x41

FUS_STATE_ADDR = 0xFFFF0054


def find_device():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        return None
    try:
        if dev.is_kernel_driver_active(INTF):
            dev.detach_kernel_driver(INTF)
    except Exception:
        pass
    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass
    return dev


def get_status(dev):
    d = dev.ctrl_transfer(REQ_IN, DFU_GETSTATUS, 0, INTF, 6, timeout=5000)
    return d[0], d[1] | (d[2] << 8) | (d[3] << 16), d[4]  # status, poll_ms, state


def prepare(dev):
    """Ensure DFU is in IDLE state (matches qFlipper's prepare())."""
    for _ in range(5):
        try:
            status, _, state = get_status(dev)
        except Exception:
            try:
                dev.ctrl_transfer(REQ_OUT, DFU_CLRSTATUS, 0, INTF, timeout=5000)
            except Exception:
                pass
            continue
        if state == 2:  # DFU_IDLE
            return True
        if status != 0:  # error
            dev.ctrl_transfer(REQ_OUT, DFU_CLRSTATUS, 0, INTF, timeout=5000)
        else:
            dev.ctrl_transfer(REQ_OUT, DFU_ABORT, 0, INTF, timeout=5000)
    _, _, state = get_status(dev)
    return state == 2


def set_address(dev, addr):
    data = struct.pack('<BI', DFUSE_SET_ADDRESS, addr)
    dev.ctrl_transfer(REQ_OUT, DFU_DNLOAD, 0, INTF, data, timeout=5000)
    status, poll_ms, state = get_status(dev)
    if poll_ms:
        time.sleep(poll_ms / 1000.0)
        get_status(dev)


def erase_page(dev, addr):
    data = struct.pack('<BI', DFUSE_ERASE_PAGE, addr)
    dev.ctrl_transfer(REQ_OUT, DFU_DNLOAD, 0, INTF, data, timeout=10000)
    status, poll_ms, state = get_status(dev)
    time.sleep(max(poll_ms / 1000.0, 0.05))
    get_status(dev)


def dfuse_write(dev, addr, data):
    """Erase and write data to flash via DfuSe protocol."""
    size = len(data)
    pages = math.ceil(size / FLASH_PAGE)

    print(f"  Erasing {pages} pages...")
    for i in range(pages):
        prepare(dev)
        erase_page(dev, addr + i * FLASH_PAGE)

    print(f"  Writing {size} bytes...")
    prepare(dev)
    set_address(dev, addr)

    xfer = 1024
    block = 2
    offset = 0
    while offset < size:
        chunk = data[offset:offset + xfer]
        dev.ctrl_transfer(REQ_OUT, DFU_DNLOAD, block, INTF, chunk, timeout=5000)
        st, poll_ms, state = get_status(dev)
        if poll_ms:
            time.sleep(poll_ms / 1000.0)
            get_status(dev)
        offset += xfer
        block += 1
    print(f"  Write complete.")


def fus_command(dev, opcode_byte):
    """Send a FUS command as block-0 DFU_DNLOAD (qFlipper protocol)."""
    prepare(dev)
    dev.ctrl_transfer(REQ_OUT, DFU_DNLOAD, 0, INTF, bytes([opcode_byte]), timeout=5000)
    try:
        st, poll_ms, state = get_status(dev)
        if poll_ms:
            time.sleep(poll_ms / 1000.0)
            while state == 4:  # DFU_DNBUSY
                st, poll_ms, state = get_status(dev)
                time.sleep(poll_ms / 1000.0)
    except usb.core.USBError:
        pass  # device may reset


def fus_get_state(dev):
    """Read FUS state via DFU_UPLOAD from 0xFFFF0054."""
    prepare(dev)
    dev.ctrl_transfer(REQ_OUT, DFU_ABORT, 0, INTF, timeout=5000)
    set_address(dev, FUS_STATE_ADDR)
    dev.ctrl_transfer(REQ_OUT, DFU_ABORT, 0, INTF, timeout=5000)
    data = dev.ctrl_transfer(REQ_IN, DFU_UPLOAD, 2, INTF, 2, timeout=5000)
    if len(data) >= 2:
        return data[0], data[1]
    return 0xFF, 0xFF


def read_sfsa(dev):
    """Read SFSA from option bytes via DFU upload."""
    prepare(dev)
    dev.ctrl_transfer(REQ_OUT, DFU_ABORT, 0, INTF, timeout=5000)
    # Option bytes at 0x1FFF8000, SFSA is in SFR at a specific offset
    # But easier: read from the FLASH_SFR register at 0x58004080
    # Actually, can't read registers via DFU. Read option bytes instead.
    # SFR option byte is at 0x1FFF8080
    set_address(dev, 0x1FFF8080)
    dev.ctrl_transfer(REQ_OUT, DFU_ABORT, 0, INTF, timeout=5000)
    data = dev.ctrl_transfer(REQ_IN, DFU_UPLOAD, 2, INTF, 4, timeout=5000)
    sfr = struct.unpack('<I', bytes(data))[0]
    sfsa = sfr & 0xFF
    return sfsa


def wait_for_dfu(timeout=60, fus=False):
    """Wait for DFU device. If CDC appears instead, send dfu command."""
    dfu_cmd = b'dfu radio\r\n' if fus else b'dfu\r\n'
    print("  Waiting for DFU device...", end='', flush=True)
    for i in range(timeout):
        time.sleep(1)
        print('.', end='', flush=True)
        dev = find_device()
        if dev:
            print(f" DFU found ({i+1}s)")
            time.sleep(1)
            return dev
        # Check if Fantasi CDC appeared instead
        import glob, serial as pyserial
        ports = glob.glob('/dev/ttyACM*')
        if ports:
            print(f" CDC on {ports[0]}, sending dfu...")
            try:
                s = pyserial.Serial(ports[0], 115200, timeout=2)
                s.dtr = True
                time.sleep(0.5)
                s.read(s.in_waiting)
                s.write(dfu_cmd)
                time.sleep(1)
                s.close()
            except Exception:
                pass
            time.sleep(5)
            dev = find_device()
            if dev:
                print(f"  DFU found after re-entry")
                time.sleep(1)
                return dev
    print(" TIMEOUT")
    return None


def cmd_start_ws():
    """Activate the wireless stack via FUS, fixing C2OPT option byte."""
    dev = find_device()
    if not dev:
        print("ERROR: no DFU device (0483:df11). Enter DFU mode first.")
        sys.exit(1)

    state, error = fus_get_state(dev)
    print(f"FUS state=0x{state:02X} error=0x{error:02X}")

    if state == 0xFE or (state == 0xFF and error == 0xFE):
        print("Wireless stack appears running already - sending 2nd GET_STATE to activate FUS...")
        try:
            state, error = fus_get_state(dev)
            print(f"FUS state=0x{state:02X} error=0x{error:02X}")
        except usb.core.USBError:
            print("Device reset (FUS activating)...")
        dev = wait_for_dfu()
        if not dev:
            sys.exit(1)
        state, error = fus_get_state(dev)
        print(f"FUS state=0x{state:02X} error=0x{error:02X}")

    print("Sending FUS_START_WS (0x5A)...")
    try:
        prepare(dev)
        fus_command(dev, 0x5A)
    except usb.core.USBError:
        pass

    print("Waiting for device to reset...")
    time.sleep(3)
    dev = find_device()
    if dev:
        state, error = fus_get_state(dev)
        print(f"Post-start FUS state=0x{state:02X} error=0x{error:02X}")

    print("Done. Flash firmware and test: fantasi> scan")


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == '--start-ws':
        cmd_start_ws()
        return

    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <stm32wb5x_BLE_*.bin>")
        print(f"       {sys.argv[0]} --start-ws   (activate wireless stack)")
        sys.exit(1)

    with open(sys.argv[1], 'rb') as f:
        binary = f.read()
    size = len(binary)
    print(f"Binary: {sys.argv[1]} ({size} bytes)")

    dev = find_device()
    if not dev:
        print("ERROR: no DFU device (0483:df11). Enter DFU mode first.")
        sys.exit(1)
    print("  DFU device found")

    pages_needed = math.ceil(size / FLASH_PAGE)

    # Step 1: Check FUS state and wait for idle
    print("\nStep 1: Checking FUS state")
    state, error = fus_get_state(dev)
    print(f"  FUS state=0x{state:02X} error=0x{error:02X}")

    if state == 0xFE or (state == 0xFF and error == 0xFE):
        print("  Wireless stack is running. Sending 2nd GET_STATE to activate FUS...")
        try:
            state, error = fus_get_state(dev)
            print(f"  FUS state=0x{state:02X} error=0x{error:02X}")
        except usb.core.USBError:
            print("  Device reset (FUS activating)...")
        dev = wait_for_dfu(fus=True)
        if not dev:
            sys.exit(1)
        state, error = fus_get_state(dev)
        print(f"  FUS state=0x{state:02X} error=0x{error:02X}")

    # Wait for any ongoing FUS operation to complete
    if state not in (0x00, 0xFF):
        print(f"  FUS busy (0x{state:02X}), waiting...")
        for w in range(60):
            time.sleep(1)
            try:
                dev_check = find_device()
                if not dev_check:
                    dev = wait_for_dfu(fus=True)
                    if not dev:
                        sys.exit(1)
                    continue
                dev = dev_check
                state, error = fus_get_state(dev)
                print(f"  [{w}s] state=0x{state:02X} error=0x{error:02X}")
                if state == 0x00:
                    break
            except usb.core.USBError:
                dev = wait_for_dfu(fus=True)
                if not dev:
                    sys.exit(1)

    if state != 0x00:
        print(f"  ERROR: FUS not idle (state=0x{state:02X} error=0x{error:02X})")
        sys.exit(1)

    print("  FUS idle, ready.")

    # Step 2: Delete old wireless stack
    print("\nStep 2: Deleting old wireless stack")
    prepare(dev)
    fus_command(dev, 0x52)  # FUS_FW_DELETE

    # Poll until complete
    for i in range(60):
        time.sleep(1)
        try:
            dev_check = find_device()
            if not dev_check:
                dev = wait_for_dfu(fus=True)
                if not dev:
                    sys.exit(1)
                continue
            dev = dev_check
            state, error = fus_get_state(dev)
            print(f"  [{i}s] state=0x{state:02X} error=0x{error:02X}")
            if state == 0x00:
                print("  Delete complete!")
                break
            if state == 0xFF:
                print(f"  FUS error: 0x{error:02X}")
                break
        except usb.core.USBError:
            print(f"  [{i}s] device resetting...")
            dev = wait_for_dfu(fus=True)
            if not dev:
                sys.exit(1)

    # Step 3: Stage binary just below FUS.
    # FUS base is at 0x080F4000 on Flipper Zero (48 KB from end of 1 MB).
    # SFSA option bytes are unreliable after delete - FUS only updates
    # them after a successful upgrade. Use fixed FUS base instead.
    FUS_BASE = 0x080F4000
    stage_addr = (FUS_BASE - size) & ~(FLASH_PAGE - 1)

    try:
        usb.util.dispose_resources(dev)
    except Exception:
        pass
    time.sleep(2)
    dev = find_device()
    if not dev:
        dev = wait_for_dfu(fus=True)
        if not dev:
            sys.exit(1)

    print(f"\nStep 3: Staging at 0x{stage_addr:08X} ({pages_needed} pages, FUS @ 0x{FUS_BASE:08X})")

    prepare(dev)
    dfuse_write(dev, stage_addr, binary)

    # Step 5: Send FUS_FW_UPGRADE
    print("\nStep 4: Sending FUS_FW_UPGRADE")
    prepare(dev)
    fus_command(dev, 0x53)

    for i in range(120):
        time.sleep(1)
        try:
            dev_check = find_device()
            if not dev_check:
                dev = wait_for_dfu(fus=True)
                if not dev:
                    sys.exit(1)
                continue
            dev = dev_check
            state, error = fus_get_state(dev)
            print(f"  [{i}s] state=0x{state:02X} error=0x{error:02X}")
            if state == 0x00:
                print("  Upgrade complete!")
                break
            if state == 0xFF and error == 0xFE:
                print("  Wireless stack is running - upgrade succeeded!")
                break
            if state == 0xFF:
                print(f"  FUS error: 0x{error:02X}")
                sys.exit(1)
        except usb.core.USBError:
            print(f"  [{i}s] device resetting...")
            dev = wait_for_dfu(fus=True)
            if not dev:
                sys.exit(1)

    # Step 6: Start wireless stack
    print("\nStep 5: Starting wireless stack")
    try:
        prepare(dev)
        fus_command(dev, 0x5A)  # FUS_START_WS
    except usb.core.USBError:
        pass

    # Leave DFU - set address to flash base then send a zero-length
    # DFU_DNLOAD which triggers the DfuSe "leave" sequence (jump to app).
    print("\nResetting into application firmware...")
    try:
        usb.util.dispose_resources(dev)
    except Exception:
        pass
    time.sleep(1)
    dev = find_device()
    if dev:
        try:
            prepare(dev)
            set_address(dev, 0x08000000)
            dev.ctrl_transfer(REQ_OUT, DFU_DNLOAD, 2, INTF, b'', timeout=5000)
            get_status(dev)
        except usb.core.USBError:
            pass

    print("Done! Check with: fantasi> radio")


if __name__ == '__main__':
    main()
