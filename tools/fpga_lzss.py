#!/usr/bin/env python3
"""Extract the configuration payload (section 'e') from a raw Xilinx .bit and
LZSS-compress it for the Proxmark3 FPGA loader (core/lzss.c decodes it).

Output: [u32 little-endian decompressed length][LZSS stream]

The compressor matches core/lzss.c's wire format: 12-bit offset (1..4096) /
4-bit length (3..18), 8 tokens per flag byte (bit 7 = first). A 4 KB window is
used so the firmware can decode with a 4 KB history buffer (the idle LF sample
buffer), no library and no full-payload RAM.

Usage: fpga_lzss.py <in.bit> <out.bit.z>
"""
import sys, struct

WIN = 4096      # max match offset (12-bit)
MIN_MATCH = 3
MAX_MATCH = 18  # 4-bit length encodes 3..18


def extract_section_e(bit: bytes) -> bytes:
    """Parse the .bit TLV (13-byte fixed header, then a-d [u16 len] and e [u32
    len]) and return section 'e' - the raw configuration frames."""
    p, n = 13, len(bit)
    while p + 5 <= n:
        tag = bit[p]; p += 1
        if tag == ord('e'):
            ln = (bit[p] << 24) | (bit[p+1] << 16) | (bit[p+2] << 8) | bit[p+3]
            p += 4
            return bit[p:p+ln]
        if not (ord('a') <= tag <= ord('d')):
            break
        ln = (bit[p] << 8) | bit[p+1]
        p += 2 + ln
    raise ValueError("no section 'e' found in .bit")


def compress(data: bytes) -> bytes:
    n = len(data)
    table = {}                 # 3-byte key -> list of positions (ascending)
    tokens = []                # ('lit', b) | ('match', offset, length)
    i = 0
    while i < n:
        best_len, best_off = 0, 0
        if i + MIN_MATCH <= n:
            key = data[i:i+3]
            for p in reversed(table.get(key, ())):
                if i - p > WIN:
                    break
                l = 0
                m = min(MAX_MATCH, n - i)
                while l < m and data[p+l] == data[i+l]:
                    l += 1
                if l > best_len:
                    best_len, best_off = l, i - p
                    if l == MAX_MATCH:
                        break
        if best_len >= MIN_MATCH:
            tokens.append(('match', best_off, best_len))
            step = best_len
        else:
            tokens.append(('lit', data[i]))
            step = 1
        for k in range(step):
            if i + k + MIN_MATCH <= n:
                table.setdefault(data[i+k:i+k+3], []).append(i + k)
        i += step

    out = bytearray()
    for j in range(0, len(tokens), 8):
        group = tokens[j:j+8]
        flag = 0
        body = bytearray()
        for t in group:
            flag <<= 1
            if t[0] == 'match':
                flag |= 1
                off, length = t[1], t[2]
                body.append((((off - 1) >> 8) << 4) | (length - MIN_MATCH))
                body.append((off - 1) & 0xFF)
            else:
                body.append(t[1])
        flag <<= (8 - len(group))          # left-align a short final group
        out.append(flag)
        out += body
    return bytes(out)


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: fpga_lzss.py <in.bit> <out.bit.z>")
    bit = open(sys.argv[1], 'rb').read()
    payload = extract_section_e(bit)
    comp = compress(payload)
    with open(sys.argv[2], 'wb') as f:
        f.write(struct.pack('<I', len(payload)))
        f.write(comp)
    total = 4 + len(comp)
    sys.stderr.write(f"{sys.argv[1]}: section-e {len(payload)} -> {total} "
                     f"({total*100//len(payload)}%)\n")


if __name__ == '__main__':
    main()
