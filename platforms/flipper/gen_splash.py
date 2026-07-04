#!/usr/bin/env python3
"""Convert a 128x64 PNG to a 1bpp ST7565 page-format bitmap.

The output is a raw 1024-byte file suitable for copying to the device's
LittleFS volume via MSC as /splash.bin.

Usage:
    python3 platforms/flipper/gen_splash.py <input.png> <output.bin>
"""

import sys
from PIL import Image

DISPLAY_WIDTH = 128
DISPLAY_HEIGHT = 64
DISPLAY_PAGES = DISPLAY_HEIGHT // 8


def png_to_st7565(path):
    img = Image.open(path).convert("L")
    if img.size != (DISPLAY_WIDTH, DISPLAY_HEIGHT):
        img = img.resize((DISPLAY_WIDTH, DISPLAY_HEIGHT))

    buf = bytearray(DISPLAY_PAGES * DISPLAY_WIDTH)
    for page in range(DISPLAY_PAGES):
        for col in range(DISPLAY_WIDTH):
            byte = 0
            for bit in range(8):
                y = page * 8 + bit
                if img.getpixel((col, y)) < 128:
                    byte |= 1 << bit
            buf[page * DISPLAY_WIDTH + col] = byte
    return bytes(buf)


def main():
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <input.png> <output.bin>", file=sys.stderr)
        sys.exit(1)

    splash = png_to_st7565(sys.argv[1])

    with open(sys.argv[2], "wb") as f:
        f.write(splash)

    print(f"{sys.argv[2]}: {len(splash)} bytes (1bpp ST7565 page format)")


if __name__ == "__main__":
    main()
