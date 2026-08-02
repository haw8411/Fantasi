#!/usr/bin/env python3
"""Unit test: the EM4100/EM410x decoder (core/rfid/rfid_em4100.c).

Compiles the real decoder TU on the host (HAL stubbed out) and runs
rfid_em4100_decode over two interval streams:

  1. A stream captured off real hardware (a Chameleon Ultra reading an EM4100
     tag whose ID is 0000001450) - the exact bytes the on-device SAADC + edge
     recovery produced. Must decode to that ID at RF/64.
  2. An IDEAL stream we generate here from a chosen ID by Manchester-encoding a
     correctly-built EM4100 frame - **even** row parity (P2..P9), even column
     parity, stop bit 0. Must round-trip.

Both matter: (1) guards the real signal path, (2) is generated independently of
the decoder. Crucially (2) uses EVEN row parity, per the EM4100 datasheet - an
earlier "synthetic" check used odd parity, the same wrong convention the decoder
had, so it validated nothing and the bug shipped. Hardware-free - safe for CI.
"""
import argparse
import os
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
DECODER = os.path.join(REPO_ROOT, "core/rfid/rfid_em4100.c")

# A real on-device capture: CU reading an EM4100 tag, ID 0000001450 (inter-edge
# intervals in carrier cycles, captured off the SAADC edge-recovery path during
# bring-up).
CAPTURE_0000001450 = [
    64,64,64,64,64,64,64,64,64,64,64,64,96,95,128,65,127,128,128,65,64,64,64,70,
    58,64,64,64,64,100,64,60,68,60,64,64,64,95,65,64,64,64,64,64,64,64,64,64,64,
    64,64,69,59,64,64,64,64,64,64,64,64,63,65,64,64,64,64,64,64,64,101,90,128,65,
    127,128,128,65,64,64,64,64,64,64,64,64,64,96,64,64,64,64,64,64,64,95,65,64,64,
    64,64,64,64,67,65,60,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
    64,64,64,
]


def skip(msg):
    print(f"SKIP: {msg}")
    sys.exit(77)


def fail(msg):
    print(f"FAIL: {msg}")
    sys.exit(1)


def build_ideal_intervals(id_hex):
    """Manchester-encode a correctly-built EM4100 frame for `id_hex` (5 bytes)
    into an ideal inter-rising-edge interval stream (carrier cycles, RF/64)."""
    idb = bytes.fromhex(id_hex)
    nibs = []
    for b in idb:
        nibs += [b >> 4, b & 0xF]
    bits = [1] * 9                       # 9-bit header
    cols = [0, 0, 0, 0]
    for nb in nibs:
        row = [(nb >> 3) & 1, (nb >> 2) & 1, (nb >> 1) & 1, nb & 1]
        bits += row + [row[0] ^ row[1] ^ row[2] ^ row[3]]   # EVEN row parity
        for i in range(4):
            cols[i] ^= row[i]
    bits += cols + [0]                   # column parity (even) + stop bit 0
    assert len(bits) == 64
    # Manchester: 0 -> level 0,1 ; 1 -> level 1,0 (the convention the decoder's
    # rising-edge state machine expects). Two frames so a full period is available
    # at any phase; interval = gap between rising edges, in cycles (half-chip = 32
    # carrier cycles).
    lvl = []
    for b in bits * 2:
        lvl += [0, 1] if b == 0 else [1, 0]
    rises = [i for i in range(1, len(lvl)) if lvl[i] == 1 and lvl[i - 1] == 0]
    return [(rises[i] - rises[i - 1]) * 32 for i in range(1, len(rises))]


HARNESS = r"""
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rfid.h"
#include "hal_rfid.h"
int hal_rfid_set_mode(rfid_mode_t m){(void)m;return 0;}
int hal_rfid_lf_field(bool on,uint32_t d){(void)on;(void)d;return 0;}
int hal_rfid_lf_acquire(uint8_t*b,int m,uint32_t o){(void)b;(void)m;(void)o;return 0;}
int rfid_em4100_decode(const uint8_t *iv, int n, uint8_t uid[5]);
int main(int argc, char **argv){
    uint8_t iv[512]; int n=0;
    for(int i=1;i<argc && n<512;i++) iv[n++]=(uint8_t)atoi(argv[i]);
    uint8_t uid[5]={0};
    int r=rfid_em4100_decode(iv,n,uid);
    printf("%d %02X%02X%02X%02X%02X\n",r,uid[0],uid[1],uid[2],uid[3],uid[4]);
    return 0;
}
"""


# rfid_em4100.c pulls in FreeRTOS.h purely for the portable allocator
# (pvPortMalloc/vPortFree; see its IV scratch buffer). On the host we have no
# FreeRTOS, so shim that one header to malloc/free - like the HAL stubs above.
FREERTOS_SHIM = r"""
#ifndef FANTASI_TEST_FREERTOS_H
#define FANTASI_TEST_FREERTOS_H
#include <stdlib.h>
static inline void *pvPortMalloc(size_t sz){ return malloc(sz); }
static inline void  vPortFree(void *p){ free(p); }
#endif
"""


def run_decode(binary, intervals):
    out = subprocess.run([binary, *[str(x) for x in intervals]],
                         capture_output=True, text=True).stdout.strip()
    rate, uid = out.split()
    return int(rate), uid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", default=None)   # ignored; accepted for the runner
    ap.parse_args()

    cc = os.environ.get("CC", "cc")
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, "harness.c")
        with open(src, "w") as f:
            f.write(HARNESS)
        with open(os.path.join(tmp, "FreeRTOS.h"), "w") as f:
            f.write(FREERTOS_SHIM)
        binary = os.path.join(tmp, "harness")
        r = subprocess.run(
            [cc, "-O1", "-I", tmp,                            # FreeRTOS.h shim
             "-I", os.path.join(REPO_ROOT, "core/rfid"),
             "-I", os.path.join(REPO_ROOT, "hal"),
             "-o", binary, src, DECODER],
            capture_output=True, text=True)
        if r.returncode != 0:
            fail(f"decoder failed to compile:\n{r.stderr}")

        # 1. real capture
        rate, uid = run_decode(binary, CAPTURE_0000001450)
        if uid != "0000001450" or rate != 64:
            fail(f"real capture: got rate={rate} uid={uid}, want 64/0000001450")
        print(f"PASS real capture -> RF/{rate} {uid}")

        # 2. ideal round-trip for a few IDs (even parity, generated independently)
        for id_hex in ("0000001450", "1234567890", "DEADBEEF01", "A5A5A5A5A5"):
            rate, uid = run_decode(binary, build_ideal_intervals(id_hex))
            if uid.upper() != id_hex.upper() or rate != 64:
                fail(f"ideal {id_hex}: got rate={rate} uid={uid}")
            print(f"PASS ideal round-trip -> RF/{rate} {uid}")

    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
