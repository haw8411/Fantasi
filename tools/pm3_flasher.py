#!/usr/bin/env python3
"""Minimal Proxmark3 bootloader flasher for Fantasi.

Speaks the PM3 bootloader's OLD packet protocol over USB CDC to write
ELF segments into the osimage region (0x102000+).  Never touches the
bootloader at 0x100000-0x101FFF.

Requires: pyserial
"""

import struct
import sys
import time

import serial

# ── PM3 bootloader protocol constants ──────────────────────────────
PACKET_SIZE      = 544          # 8 + 24 + 512
PM3_CMD_DATA_SIZE = 512
BLOCK_SIZE       = 0x200        # 512 bytes = 2 flash pages

CMD_DEVICE_INFO    = 0x0000
CMD_FINISH_WRITE   = 0x0003
CMD_HARDWARE_RESET = 0x0004
CMD_START_FLASH    = 0x0005
CMD_CHIP_INFO      = 0x0006
CMD_ACK            = 0x00FF
CMD_NACK           = 0x00FE

FLASH_START      = 0x100000
BOOTLOADER_END   = 0x102000

DEVICE_INFO_FLAG_CURRENT_MODE_BOOTROM  = 1 << 2
DEVICE_INFO_FLAG_UNDERSTANDS_START_FLASH = 1 << 4
DEVICE_INFO_FLAG_UNDERSTANDS_CHIP_INFO   = 1 << 5

# ── Packet helpers ─────────────────────────────────────────────────
def _pack_cmd(cmd, arg0=0, arg1=0, arg2=0, data=b""):
    """Build a 544-byte PacketCommandOLD."""
    hdr = struct.pack("<QQQQ", cmd, arg0, arg1, arg2)  # 32 bytes
    payload = data.ljust(PM3_CMD_DATA_SIZE, b"\x00")[:PM3_CMD_DATA_SIZE]
    return hdr + payload

def _read_response(ser, timeout=5.0):
    """Read one 544-byte PacketResponseOLD, return (cmd, arg0, arg1, arg2, data)."""
    deadline = time.monotonic() + timeout
    buf = b""
    while len(buf) < PACKET_SIZE:
        remaining = PACKET_SIZE - len(buf)
        left = deadline - time.monotonic()
        if left <= 0:
            raise TimeoutError(f"Timeout reading response (got {len(buf)}/{PACKET_SIZE} bytes)")
        ser.timeout = min(left, 1.0)
        chunk = ser.read(remaining)
        if not chunk:
            continue
        buf += chunk
    cmd, a0, a1, a2 = struct.unpack_from("<QQQQ", buf, 0)
    data = buf[32:]
    return cmd, a0, a1, a2, data

# ── ELF loader (minimal, PT_LOAD only) ────────────────────────────
ELF_MAGIC = b"\x7fELF"

def _load_elf_segments(path):
    """Return list of (paddr, data) for PT_LOAD segments with nonzero filesz."""
    with open(path, "rb") as f:
        elf = f.read()

    if elf[:4] != ELF_MAGIC:
        raise ValueError("Not an ELF file")

    ei_class = elf[4]
    if ei_class != 1:
        raise ValueError("Not a 32-bit ELF")

    e_phoff = struct.unpack_from("<I", elf, 28)[0]
    e_phentsize = struct.unpack_from("<H", elf, 42)[0]
    e_phnum = struct.unpack_from("<H", elf, 44)[0]

    segments = []
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        p_type, p_offset, _vaddr, p_paddr, p_filesz, p_memsz = struct.unpack_from("<IIIIII", elf, off)

        PT_LOAD = 1
        if p_type != PT_LOAD or p_filesz == 0:
            continue

        data = elf[p_offset : p_offset + p_filesz]
        segments.append((p_paddr, data))

    return segments

def _merge_and_block_align(segments):
    """Merge adjacent segments that share a block boundary, pad to BLOCK_SIZE,
    and return list of (start_addr, data) ready to flash."""
    if not segments:
        return []

    segments.sort(key=lambda s: s[0])

    merged = []
    cur_start, cur_data = segments[0]

    for paddr, data in segments[1:]:
        cur_end = cur_start + len(cur_data)
        # same block boundary → merge
        if (paddr & ~(BLOCK_SIZE - 1)) == ((cur_end - 1) & ~(BLOCK_SIZE - 1)):
            gap = paddr - cur_end
            cur_data = cur_data + b"\xff" * gap + data
        else:
            merged.append((cur_start, cur_data))
            cur_start, cur_data = paddr, data

    merged.append((cur_start, cur_data))

    # block-align each segment
    aligned = []
    for start, data in merged:
        block_off = start & (BLOCK_SIZE - 1)
        if block_off:
            data = b"\xff" * block_off + data
            start -= block_off
        # pad tail to block boundary
        tail = len(data) % BLOCK_SIZE
        if tail:
            data += b"\xff" * (BLOCK_SIZE - tail)
        aligned.append((start, data))

    return aligned

# ── Flash orchestration ────────────────────────────────────────────
def flash(port_path, elf_path):
    segments = _load_elf_segments(elf_path)
    if not segments:
        print("error: no loadable segments in ELF", file=sys.stderr)
        return 1

    segs = _merge_and_block_align(segments)

    # Validate all segments are in the osimage region
    for start, data in segs:
        end = start + len(data)
        if start < BOOTLOADER_END:
            print(f"error: segment at 0x{start:08x} overlaps bootloader region", file=sys.stderr)
            return 1

    print(f"Opening {port_path}...")
    ser = serial.Serial(port_path, baudrate=115200, timeout=2)

    # Drain any stale data
    ser.reset_input_buffer()

    # 1. Query device state
    ser.write(_pack_cmd(CMD_DEVICE_INFO))
    cmd, flags, _, _, _ = _read_response(ser)
    if cmd not in (CMD_DEVICE_INFO, CMD_ACK):
        print(f"error: unexpected response 0x{cmd:04x} to DEVICE_INFO", file=sys.stderr)
        ser.close()
        return 1

    if not (flags & DEVICE_INFO_FLAG_CURRENT_MODE_BOOTROM):
        print("error: device is not in bootloader mode", file=sys.stderr)
        ser.close()
        return 1

    print("Bootloader detected.")

    # 2. Get chip info for flash size
    flash_size = 512 * 1024  # default to 512K
    if flags & DEVICE_INFO_FLAG_UNDERSTANDS_CHIP_INFO:
        ser.write(_pack_cmd(CMD_CHIP_INFO))
        cmd, cidr, _, _, _ = _read_response(ser)
        if cmd == CMD_CHIP_INFO:
            nvpsiz = (cidr >> 8) & 0xF
            sizes = {0: 0, 1: 8, 2: 16, 3: 32, 5: 64, 7: 128, 9: 256, 10: 512, 12: 1024}
            flash_size = sizes.get(nvpsiz, 512) * 1024
            print(f"Flash size: {flash_size // 1024}K")

    flash_end = FLASH_START + flash_size

    # Validate segments fit in flash
    for start, data in segs:
        if start + len(data) > flash_end:
            print(f"error: segment at 0x{start:08x} exceeds flash end 0x{flash_end:08x}", file=sys.stderr)
            ser.close()
            return 1

    # 3. Send START_FLASH (no bootloader unlock)
    if flags & DEVICE_INFO_FLAG_UNDERSTANDS_START_FLASH:
        ser.write(_pack_cmd(CMD_START_FLASH, BOOTLOADER_END, flash_end, 0))
        cmd, _, _, _, _ = _read_response(ser)
        if cmd == CMD_NACK:
            print("error: START_FLASH rejected", file=sys.stderr)
            ser.close()
            return 1

    # 4. Write blocks
    total_blocks = sum(len(d) // BLOCK_SIZE for _, d in segs)
    written = 0

    for start, data in segs:
        nblocks = len(data) // BLOCK_SIZE
        end = start + len(data)
        print(f"  0x{start:08x}..0x{end - 1:08x} [{len(data)} bytes / {nblocks} blocks]")

        for i in range(nblocks):
            block_addr = start + i * BLOCK_SIZE
            block_data = data[i * BLOCK_SIZE : (i + 1) * BLOCK_SIZE]

            ser.write(_pack_cmd(CMD_FINISH_WRITE, block_addr, 0, 0, block_data))
            cmd, a0, _, _, _ = _read_response(ser, timeout=10.0)

            if cmd != CMD_ACK:
                print(f"\nerror: write failed at 0x{block_addr:08x} (response 0x{cmd:04x})", file=sys.stderr)
                if cmd == CMD_NACK and a0:
                    if a0 & 0x04:
                        print("  lock error", file=sys.stderr)
                    if a0 & 0x08:
                        print("  programming error", file=sys.stderr)
                ser.close()
                return 1

            written += 1
            sys.stdout.write(".")
            sys.stdout.flush()

    print(f" ok ({written} blocks)")

    # 5. Reset
    ser.write(_pack_cmd(CMD_HARDWARE_RESET))
    ser.close()
    print("Flash complete.")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <serial-port> <firmware.elf>", file=sys.stderr)
        sys.exit(1)

    sys.exit(flash(sys.argv[1], sys.argv[2]))
