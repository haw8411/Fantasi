# Flashing

Every target has two paths: a **USB path** (the blessed, user-facing flow that goes
through the device's own bootloader) and an **SWD/JTAG path** (for bring-up, dead
firmware recovery, or CI). USB paths are preferred - each is designed so a bad
image can't brick the device. SWD paths have target-specific landmines and must
be run with care.

All four devices run the same Fantasi CLI once flashed. Connect at 115200 baud
to the enumerated CDC port (`/dev/ttyACM*` on Linux, `/dev/tty.usbmodem*` on macOS):

```
fantasi> device
FZ      # or KIISU or CU or PM3
```

## Auto-detection

`make flash` with no `PLATFORM` auto-detects the single connected board - by
querying a running Fantasi's `device` id over CDC, or by spotting a board already
sitting in its DFU mode (each has a unique DFU VID:PID, see the quick reference
below). If more than one device is connected it **refuses to flash** rather than
guess the target; pass `PLATFORM=<name>` to disambiguate. The same safety applies
when `PLATFORM` is set: the flasher only talks to the board whose `device` id
matches, so a second connected board can't be flashed by mistake.

```
make flash                  # auto-detect + flash the one connected device
make PLATFORM=flipper flash  # force a specific target
python3 tools/flash.py --detect   # print the detected platform and exit
```

## Flipper Zero (STM32WB55)

### USB - ST ROM DFU

The STM32WB55 has a ROM-resident DFU bootloader in system memory at `0x1FFF0000`.
It is physically separate from user flash and cannot be erased by `dfu-util`, so
this path is always a safe recovery.

1. Build and flash: `make PLATFORM=flipper flash`

If the device is already running Fantasi, the unified flasher (`tools/flash.py`)
automatically uploads resources via MSC, sends `dfu` over CDC serial, and flashes
with `dfu-util`. On first flash, it prompts for manual DFU entry.

Manual DFU entry:
- From a running Fantasi: `fantasi> dfu`
- From any Flipper firmware: hold **OK + BACK** for 30 s while the cable is
  disconnected (must be charged)
- Verify: `lsusb | grep 0483:df11`

Internally `tools/flash.py` runs:

```
dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D build/flipper/fantasi-flipper.bin
```

The `:leave` suffix tells the ROM to detach and reset into user flash when the
transfer finishes, so the new firmware comes up immediately.

### SWD - Black Magic Probe

Wire BMP to the Flipper's SWD test pads and attach via GDB:

```
arm-none-eabi-gdb \
  -ex 'target extended-remote /dev/ttyACM1' \
  -ex 'monitor swdp_scan' \
  -ex 'attach 1' \
  -ex 'monitor erase_mass' \
  -ex 'load build/flipper/fantasi-flipper.elf' \
  -ex 'kill'
```

This erases all user flash and reprograms it. The ROM DFU at `0x1FFF0000` is
read-only and untouched - recovery is still `dfu-util`.

## Kiisu (STM32WB55)

The Kiisu is the same STM32WB55 MCU as the Flipper Zero and flashes exactly like
it. Build and flash with `make PLATFORM=kiisu flash`; the bin is at
`build/kiisu/fantasi-kiisu.bin`. The unified flasher uses `dfu-util` against the
STM32 ROM DFU (`0483:df11`, ROM-resident at `0x1FFF0000`, app base `0x08000000`),
so both the USB and SWD paths are identical to the [Flipper Zero](#flipper-zero-stm32wb55)
section above - substitute the Kiisu bin/elf paths.

## Proxmark3 (AT91SAM7S)

### USB - PM3 bootloader

The PM3 bootloader lives in the first 8 KB of flash (`0x100000-0x101FFF`) and is
the only recovery path without a JTAG adapter. The bundled flasher
(`tools/pm3_flasher.py`) never writes to that region.

1. Build and flash: `make PLATFORM=proxmark3 flash`

If the device is already running Fantasi, the unified flasher sends `dfu` over
CDC serial and waits for the bootloader to enumerate. On first flash, it prompts
for manual bootloader entry.

Manual bootloader entry:
- From a running Fantasi: `fantasi> dfu`
- From any PM3 firmware: hold the PM3 button while plugging in USB
- Verify: `lsusb | grep 9ac4:4b8f`

Internally `tools/flash.py` runs `tools/pm3_flasher.py` (requires `pyserial`),
which writes only the `osimage` region starting at `0x102000`.

### JTAG - CMSIS-DAP (dangerous - read first)

AT91SAM7S flash has **16 KB lock regions** aligned at `0x100000`. The first
region spans `0x100000-0x103FFF` and therefore covers both the bootloader
(`0x100000-0x101FFF`) and the first 8 KB of the Fantasi osimage
(`0x102000-0x103FFF`). OpenOCD's sam7 flash driver must erase a full lock
region to write any page in it, so naively flashing Fantasi alone erases the
bootloader as collateral damage.

**Always flash bootrom + Fantasi together in one openocd session:**

```bash
openocd -f interface/cmsis-dap.cfg \
        -c 'transport select jtag; adapter speed 2000' \
        -f target/at91sam7sx.cfg -c '
  init; reset halt
  flash write_image erase /path/to/bootrom/obj/bootrom.elf
  flash write_image /path/to/fantasi-proxmark3.elf
  verify_image /path/to/bootrom/obj/bootrom.elf
  verify_image /path/to/fantasi-proxmark3.elf
  reset run
  exit'
```

If the bootrom ever does get erased, recovery requires either the **ERASE pad**
(shorts `GPNVM0`, forces boot into the on-chip SAM-BA ROM) or a full JTAG replay
of the bootrom ELF.

OpenOCD may log `Adding extra erase range, 0x00100000 .. 0x00101fff` - that's
the driver correctly extending the erase to the lock-region boundary. It's the
reason bootrom must be re-written in the same transaction.

### Post-reset note

The PM3 USB D+ pullup on this hardware is wired permanently to 3V3, so a CPU
reset (via `reboot` or the RSTC) does **not** cause the host to see a USB
disconnect. The CDC stays enumerated with stale state and won't immediately
respond. If the host stalls, unplug and replug.

## Chameleon Ultra (nRF52840)

### USB - signed DFU via nrfutil

The stock Chameleon Ultra ships with a Nordic Secure DFU bootloader around
`0xF3000`. It requires **cryptographically signed** packages
(`NRF_DFU_REQUIRE_SIGNED_APP_UPDATE=1` in the upstream config); unsigned
packages are silently rejected. The matching private key is published by the
ChameleonUltra project at `resource/dfu_key/chameleon.pem` and vendored in this
repo as `platforms/chameleon/dfu_key.pem`.

1. Build and flash: `make PLATFORM=chameleon flash`

If the device is already running Fantasi, the unified flasher sends `dfu` over
CDC serial and waits for the Nordic DFU bootloader to enumerate. On first flash,
it prompts for manual DFU entry.

Manual DFU entry:
- From a running Fantasi: `fantasi> dfu`
- From any firmware: hold the user button while plugging in USB
- Verify: `lsusb | grep 1915:521f`

Internally `tools/flash.py` packages a signed DFU zip and flashes via nrfutil:

```
nrfutil pkg generate \
  --hw-version 0 --sd-req 0x00 --application-version 1 \
  --application build/chameleon/fantasi-chameleon.hex \
  --key-file platforms/chameleon/dfu_key.pem \
  build/chameleon/fantasi-chameleon-dfu.zip
nrfutil dfu usb-serial -pkg <zip> -p /dev/ttyACM<n>
```

**30-second inactivity timeout.** The bootloader is built with
`NRF_BL_DFU_INACTIVITY_TIMEOUT_MS=30000`. If no DFU protocol traffic arrives
within 30 s of entering DFU mode, the bootloader chains back to the valid app.
Actual DFU traffic resets the timer, so a real flash doesn't hit it - just
don't dawdle between "enter DFU" and starting `nrfutil`.

### SWD - CMSIS-DAP (requires a matching settings page)

The bootloader settings page at `0xFF000` contains `boot_validation_app`, an
ECDSA-P256-SHA256 hash of the installed app. When you flash a new Fantasi over
SWD the old hash no longer matches, and the bootloader refuses to chain to the
new app - the device stays enumerated as `1915:521f` (the bootloader's own USB
identity in its "wait" state), and a `halt` via openocd shows PC in the
`0xF3000+` range.

The fix is to regenerate the settings page with `nrfutil` and flash it in the
same session:

```bash
BUILD=build/chameleon
nrfutil settings generate --family NRF52840 \
  --application        $BUILD/fantasi-chameleon.hex \
  --application-version 1 \
  --bootloader-version  2 \
  --bl-settings-version 2 \
  --app-boot-validation VALIDATE_ECDSA_P256_SHA256 \
  --key-file            platforms/chameleon/dfu_key.pem \
  $BUILD/bl_settings.hex

openocd -c 'adapter driver cmsis-dap' \
        -c 'transport select swd' \
        -c 'adapter speed 1000' \
        -f target/nrf52.cfg -c "
  init; halt
  program $BUILD/fantasi-chameleon.hex verify
  program $BUILD/bl_settings.hex        verify reset
  exit"
```

The app is linked at `0x27000`; MBR (`0x0`), SoftDevice (`0x1000-0x27000`), and
bootloader (`0xF3000+`) are preserved by never writing outside `[0x27000,
0x2Cxxx]` and the single 4 KB page at `0xFF000`.

## Quick reference

| Target | USB bootloader VID:PID | App base | Bootloader region preserved | SWD landmine |
|---|---|---|---|---|
| Flipper Zero | `0483:df11` (STM ROM DFU) | `0x08000000` | ROM `0x1FFF0000` (read-only, can't brick) | none |
| Proxmark3 | `9ac4:4b8f` (PM3 bootloader) | `0x00102000` | `0x100000-0x101FFF` | 16 KB lock-region alignment erases bootrom - flash bootrom + app together |
| Chameleon Ultra | `1915:521f` (Nordic Secure DFU) | `0x00027000` | `0xF3000+` | Bootloader settings page at `0xFF000` must match the app hash - regenerate with `nrfutil settings generate` |
