#!/usr/bin/env python3
"""Generate font_data.c/.h from u8g2 font blobs in the stock Flipper firmware.

Decodes the u8g2-compressed fonts the stock Flipper UI uses and re-emits them
as plain column bitmaps (fz_font_t in display.h): per glyph a small header
(width, advance, x/y offset) plus one 16-bit column per pixel column, bit 0 =
top row of the glyph's own bounding box. Uncompressed costs ~1.5 KB per font
and keeps the firmware-side renderer trivial.

The generated font_data.c is committed, so this only needs re-running to add
glyphs or swap fonts:

    python3 platforms/flipper/gen_font.py \
        [path/to/u8g2_fonts.c]  (default: original_fw/.../lib/u8g2/u8g2_fonts.c)
"""

import codecs
import os
import re
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_SRC = os.path.join(
    REPO, "original_fw", "flipperzero-firmware", "lib", "u8g2", "u8g2_fonts.c")
OUT_C = os.path.join(REPO, "platforms", "flipper", "font_data.c")

# (u8g2 name, C symbol)
FONTS = [
    ("u8g2_font_haxrcorp4089_tr", "font_item"),   # stock FZ menu font
    ("u8g2_font_helvB08_tr",      "font_bold"),   # stock FZ title font
]

FIRST, LAST = 32, 128   # ASCII + 0x7F up-arrow + 0x80 down-arrow

# Hand-authored arrows (no unicode glyphs in the _tr fonts). Bit 0 = glyph
# top row. The up arrow sits a row lower than the caps so it reads optically
# centered next to lowercase text (the splash button label).
ARROWS = {
    0x7F: dict(w=5, adv=6, xoff=0, ytop_from_asc=2, cols=[0x10, 0x1C, 0x1F, 0x1C, 0x10]),
    0x80: dict(w=5, adv=6, xoff=0, ytop_from_asc=1, cols=[0x01, 0x07, 0x1F, 0x07, 0x01]),
}


def extract_font(text, name):
    m = re.search(r"const uint8_t " + re.escape(name)
                  + r'\[\d+\][^=]*=\s*((?:"(?:[^"\\]|\\.)*"\s*)+);', text, re.S)
    if not m:
        raise KeyError(name)
    blob = b""
    for lit in re.findall(r'"((?:[^"\\]|\\.)*)"', m.group(1), re.S):
        blob += codecs.escape_decode(lit.encode("latin-1"))[0]
    return blob


class U8g2Font:
    def __init__(self, data):
        d = self.data = data
        (self.glyph_cnt, self.bbx_mode, self.bits_per_0, self.bits_per_1,
         self.bpcw, self.bpch, self.bpcx, self.bpcy, self.bpdx) = d[0:9]
        self.start_A = (d[17] << 8) | d[18]
        self.start_a = (d[19] << 8) | d[20]

    def glyph_offset(self, code):
        f = 23
        if code >= ord('a'):
            f += self.start_a
        elif code >= ord('A'):
            f += self.start_A
        d = self.data
        while True:
            if d[f + 1] == 0:
                return None
            if d[f] == code:
                return f + 2
            f += d[f + 1]

    def decode(self, code):
        """-> dict(w, h, x, y, adv, px=set of (x,y) in glyph bbox) or None."""
        off = self.glyph_offset(code)
        if off is None:
            return None
        pos = [off, 0]

        def bits(cnt):
            byi, bip = pos
            val = self.data[byi] >> bip
            if bip + cnt >= 8:
                val |= self.data[byi + 1] << (8 - bip)
                pos[0] += 1
            pos[1] = (bip + cnt) % 8
            return val & ((1 << cnt) - 1)

        def sbits(cnt):
            return bits(cnt) - (1 << (cnt - 1))

        w = bits(self.bpcw)
        h = bits(self.bpch)
        x = sbits(self.bpcx)
        y = sbits(self.bpcy)
        adv = sbits(self.bpdx)
        px = set()
        if w > 0:
            p = 0
            while p < w * h:
                a = bits(self.bits_per_0)
                b = bits(self.bits_per_1)
                while True:
                    p += a
                    for i in range(b):
                        px.add(((p + i) % w, (p + i) // w))
                    p += b
                    if bits(1) == 0:
                        break
        return dict(w=w, h=h, x=x, y=y, adv=adv, px=px)


def build(u8g2, sym):
    glyphs = {}
    for code in range(FIRST, LAST + 1):
        if code in ARROWS:
            continue
        g = u8g2.decode(code)
        if g:
            glyphs[code] = g

    # Baseline geometry from the actual glyph extents: in u8g2 a glyph's top
    # row sits at baseline-(h+y), its bottom row at baseline-(y+1).
    ascent = max(g["h"] + g["y"] for g in glyphs.values() if g["w"])
    descent = max(-(g["y"]) for g in glyphs.values() if g["w"])
    if descent < 0:
        descent = 0

    entries = []   # (code, w, adv, xoff, ytop, cols[])
    for code in range(FIRST, LAST + 1):
        if code in ARROWS:
            a = ARROWS[code]
            entries.append((code, a["w"], a["adv"], a["xoff"],
                            a["ytop_from_asc"], a["cols"]))
            continue
        g = glyphs.get(code)
        if not g:
            entries.append((code, 0, 4, 0, 0, []))
            continue
        ytop = ascent - g["h"] - g["y"]
        cols = []
        for i in range(g["w"]):
            v = 0
            for r in range(g["h"]):
                if (i, r) in g["px"]:
                    v |= 1 << r
            cols.append(v)
        entries.append((code, g["w"], g["adv"], g["x"], ytop, cols))

    col_blob, structs = [], []
    for code, w, adv, xoff, ytop, cols in entries:
        off = len(col_blob)
        col_blob.extend(cols)
        ch = chr(code) if 32 < code < 127 and code != 92 else "0x%02X" % code
        structs.append(f"    {{ {w}, {adv}, {xoff}, {ytop}, {off} }},"
                       f" /* {ch} */")

    lines = []
    lines.append(f"static const uint16_t {sym}_cols[{len(col_blob)}] = {{")
    for i in range(0, len(col_blob), 12):
        lines.append("    " + ", ".join("0x%04X" % v
                                        for v in col_blob[i:i + 12]) + ",")
    lines.append("};")
    lines.append("")
    lines.append(f"static const fz_glyph_t {sym}_glyphs[] = {{")
    lines.extend(structs)
    lines.append("};")
    lines.append("")
    lines.append(f"const fz_font_t {sym} = {{ {ascent}, {descent}, "
                 f"{FIRST}, {LAST}, {sym}_glyphs, {sym}_cols }};")
    lines.append("")
    print(f"{sym}: ascent={ascent} descent={descent} "
          f"cols={len(col_blob)*2}B glyphs={len(entries)*6}B")
    return "\n".join(lines)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SRC
    text = open(src, encoding="utf-8", errors="replace").read()

    parts = [
        "/* Generated by gen_font.py - do not edit by hand.",
        " *",
        " * Stock Flipper UI fonts, decoded from u8g2 blobs into plain column",
        " * bitmaps: font_item = haxrcorp 4089 (menu rows), font_bold =",
        " * Helvetica Bold 8 (titles). See fz_font_t in display.h. */",
        "",
        '#include "display.h"',
        "",
    ]
    for name, sym in FONTS:
        parts.append(build(U8g2Font(extract_font(text, name)), sym))
    with open(OUT_C, "w") as f:
        f.write("\n".join(parts))
    print(f"wrote {OUT_C}")


if __name__ == "__main__":
    main()
