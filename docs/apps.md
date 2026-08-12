# Apps

Fantasi can load and run small, statically-compiled **apps** from RAM. An app is
an ELF that the firmware relocates into RAM, runs on its own interruptible task,
and frees completely when it exits.

Supported on all five targets: Flipper Zero, Kiisu, Chameleon Ultra, and Proxmark5
(Cortex-M4 / ARMv7E-M) and the Proxmark3 (ARM7TDMI / ARMv4T). The loader handles both
architectures; apps are **architecture-specific**, so build and upload the
variant that matches your target (see below).

## Where apps live

| Path | Backing | Persists across reboot | Notes |
|---|---|---|---|
| `/ramfs/<name>` | RAM | No | Stored verbatim/contiguous; loaded with no flash access |
| `/apps/<name>`  | Internal flash (LittleFS) | Yes | `/apps` is created automatically at boot |

Both are exposed over USB as **one FAT drive** (see below) and over BLE via the
host CLI. `launch` works from either location.

## Writing and building an app

An app is a freestanding C file with a single entry point. It receives a
`fantasi_api_t` table (console, heap, storage, hardware) and returns an exit
code:

```c
#include "app_api.h"

int app_main(const fantasi_api_t *api)
{
    api->print("hello\n");
    void *p = api->malloc(64);      /* freed automatically at exit if you don't */
    api->free(p);
    return 0;                       /* reported as the exit code */
}
```

Put it at `apps/<name>/<name>.c` and build the loadable ELFs:

```
make app APP=<name>        # -> build/apps/<name>.cm4.elf  (Flipper, Kiisu, Chameleon, Proxmark5)
                           #    build/apps/<name>.arm7.elf (Proxmark3)
```

One build produces a variant per architecture; upload the one matching your
target. See `apps/app_api.h` for the full API; a pointer is `NULL` where the
device lacks that capability (e.g. the display on a device without a screen), so
check before calling.

Under the hood apps are built `-ffreestanding -nostdlib -mword-relocations
-mlong-calls` and linked `-r` with `apps/app.ld` into a relocatable object. Those
flags force every address reference through a relocated literal, so the loader
only ever sees `R_ARM_ABS32` (plus, on ARM7TDMI, the `R_ARM_V4BX` BX-marker,
which it ignores - AT91SAM7S has a real `BX`). The firmware loader copies the
sections into RAM, applies relocations, resolves a few compiler-runtime helpers,
and calls `app_main`.

## Getting an app onto the device

### One step: `make launch` (recommended)

`make launch APP=<name>` does the whole loop - builds the app for the connected
board, uploads it to `/ramfs/<name>`, and opens an interactive `launch` session:

```
make launch APP=hello
```

It auto-detects the target (so it picks the right `.cm4`/`.arm7` variant); force
one with `make PLATFORM=<flipper|kiisu|chameleon|proxmark3|proxmark5> launch APP=<name>`. The
app's output streams to your terminal; press **Ctrl-C** to stop it - that's
forwarded to the device as a clean kill (task deleted, memory freed) and drops
you back at the fantasi prompt. (Under the hood this is `tools/launch.py`; the host
CLI must be built, which `make` handles.)

To reach the device over **BLE** instead of USB (Flipper/Kiisu/Chameleon), add `BLE=1`:

```
make launch APP=hello BLE=1                         # auto-discover the device
make launch APP=hello BLE=AA:BB:CC:DD:EE:FF         # pick one (if several bonded)
```

The device must already be bonded (pair it once with `build/cli/fantasi --ble`);
the passkey is shown on the device screen / its `log` output.

### Over USB (mount the drive)

The device presents a small FAT drive with `RAMFS/` and `APPS/` folders. Copy an
app in and launch it:

```
# pick the variant for your target: .cm4.elf (FZ/Kiisu/CU/PM5) or .arm7.elf (PM3)
cp build/apps/hello.cm4.elf  /media/<you>/Fantasi/RAMFS/hello   # RAM (no flash wear)
# or
cp build/apps/hello.cm4.elf  /media/<you>/Fantasi/APPS/hello    # flash (persistent)
sync
```

(Mountpoint varies by OS/automounter; `lsblk` shows the `Fantasi` FAT volume.)

### Over USB serial / BLE (host CLI)

```
build/cli/fantasi                      # USB, or --ble-addr=<addr> for BLE
fantasi> upload build/apps/hello.arm7.elf /ramfs/hello   # .arm7.elf for a Proxmark3
fantasi> ls /ramfs
```

## Running

`launch` is a device command (works over USB serial and BLE):

```
fantasi> launch /ramfs/hello
hello
exit 0
```

The app runs on a dedicated task; the CLI/BLE stays responsive. Press **Ctrl-C**
to stop a running app - its task is killed and all of its memory is freed.
Verify with `free` before and after: the heap returns to its pre-launch value.
(`make launch APP=<name>`, above, builds, uploads, and runs this in one step.)

Apps that need uninterruptible timing can wrap a region in
`api->critical_enter()` / `api->critical_exit()`.

## Memory

The app image, its task stack, and anything it allocated via `api->malloc` all
come from the firmware heap and are reclaimed when the app exits or is killed -
even if the app leaks. `free` reflects the result.
