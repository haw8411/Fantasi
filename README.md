<p align="center">
  <img src="art/fantasi.png" alt="Fantasi" width="60%"><br>
  <img src="https://readme-typing-svg.herokuapp.com?font=Fira+Code&size=16&duration=1&pause=1000&color=3900FF&repeat=false&width=295&height=35&lines=█▓▒░+All+of+your+devices+-+one" alt="Typing SVG" /></a> <img src="https://readme-typing-svg.herokuapp.com?font=Fira+Code&pause=1000&color=E44AF7&duration=2000&width=90&height=35&size=16&baseline=50&lines=fantasi" style="vertical-align: middle;" alt="Typing SVG" />
</p>


## Supported targets

| Target | MCU | Flash tool |
|---|---|---|
| Flipper Zero | STM32WB55 (Cortex-M4) | `dfu-util` via ROM DFU |
| Kiisu | STM32WB55 (Cortex-M4)| `dfu-util` via ROM DFU |
| Chameleon Ultra | nRF52840 (Cortex-M4) | `nrfutil` DFU |
| Proxmark3 | AT91SAM7S512 (ARM7TDMI) | `tools/pm3_flasher.py` (pyserial) |
| Proxmark5 | AT32F435 (Cortex-M4) | `dfu-util` via AT32 ROM DFU |
| T-Watch Ultra | ESP32-S3 | Coming soon |
| Kode Dot | ESP32-S3 | Coming soon |

## Build

```
make                        # build every firmware + the host CLI
make PLATFORM=flipper       # STM32WB55 (Cortex-M4)
make PLATFORM=kiisu         # STM32WB55 (Cortex-M4)
make PLATFORM=chameleon     # nRF52840  (Cortex-M4)
make PLATFORM=proxmark3     # AT91SAM7S (ARM7TDMI)
make PLATFORM=proxmark5     # AT32F435  (Cortex-M4)
make cli                    # host CLI only (build/cli/fantasi)
make clean                  # remove all build artifacts (or PLATFORM=<x> for one)
make help                   # list targets
```

A bare `make` builds every firmware and the host CLI; `make clean` wipes the
`build/` tree. Pass `PLATFORM=<name>` to scope to a single firmware, or
`make cli` to build only the host CLI.

The firmware builds need `arm-none-eabi-gcc` and `git` on `PATH`; TinyUSB and
LittleFS are auto-cloned on demand (pinned to known-good tags). The host CLI is
built with the system `cc` and needs `libreadline` (and `libsystemd` for BLE).

Artifacts land in `build/`:

| Target | Output |
|---|---|
| flipper | `build/flipper/fantasi-flipper.{elf,bin}` |
| kiisu | `build/flipper/fantasi-kiisu.{elf,bin}` |
| chameleon | `build/chameleon/fantasi-chameleon.{elf,bin,hex}` |
| proxmark3 | `build/proxmark3/fantasi-proxmark3.{elf,bin}` |
| proxmark5 | `build/proxmark5/fantasi-proxmark5.{elf,bin}` |
| host CLI | `build/cli/fantasi` |

## Flash

```
make flash                  # auto-detect the connected device + flash it
make PLATFORM=<name> flash  # force a specific target
```

With no `PLATFORM`, `make flash` auto-detects the single connected board - by
querying a running Fantasi's `device` id, or by spotting a board already sitting
in its DFU mode. If more than one device is connected it **refuses to flash**
rather than guess; pass `PLATFORM=<name>` to disambiguate.

A successful flash is indicated by a solid blue LED.

If the device is already running Fantasi, the flasher (`tools/flash.py`)
uploads changed resources via MSC, sends `dfu` over CDC serial, and flashes with
the platform's tool. On first flash it falls back to a manual DFU prompt. Every
USB flash path preserves the device's own bootloader, so a bad image can't brick
the board.

See [docs/flashing.md](docs/flashing.md) for USB and SWD/JTAG flows, per-target
landmines, and recovery notes.

## Host CLI (recommended)

`build/cli/fantasi` is a host-side shell that talks to a device over USB serial
or BLE and gives you a filesystem view of its flash (LittleFS).

A bare `make` builds it; to build only the host tool use `make cli` (needs
libreadline; libsystemd enables BLE).

```
build/cli/fantasi                       # auto-find a device over USB, else BLE
build/cli/fantasi /dev/ttyACM0          # pin a serial port
build/cli/fantasi --ble                 # connect over BLE (first paired device)
build/cli/fantasi --ble-addr=F1:50:0F:05:34:62   # connect to a specific address
```

Inside the shell: `ls`, `cd`, `pwd`, `cat`, `upload <local> [remote]`, `mkdir`,
`rm`, `rmdir`, `help`, `exit`. Over BLE it pairs on demand - enter the passkey
shown in the device's USB log. See [docs/bluetooth.md](docs/bluetooth.md) for
pairing details and [docs/cli.md](docs/cli.md) for the full command reference.

## Apps (all targets)

Fantasi loads and runs small statically-compiled ELF apps from RAM - on the
Flipper Zero, Kiisu, Chameleon Ultra, Proxmark5 (Cortex-M4), **and** the
Proxmark3 (ARM7TDMI).
Build one with `make app APP=<name>` (source at `apps/<name>/<name>.c`); this
produces a per-architecture variant (`build/apps/<name>.cm4.elf` for
FZ/CU/KIISU/PM5, `<name>.arm7.elf` for PM3). Copy the matching variant to the device's `/ramfs`
(RAM) or `/apps` (flash) - over USB the device mounts as a FAT drive with
`RAMFS/` and `APPS/` folders, or over BLE/serial via the host CLI `upload` -
then run it:

```
fantasi> launch /ramfs/hello
```

Apps run on a dedicated task (Ctrl-C stops and frees them) and get a small API
(console, heap, storage, hardware) via `app_main(api)`. See [docs/apps.md](docs/apps.md).

## Device CLI

Connect at 115200 baud to the enumerated CDC port (`/dev/ttyACM*` on Linux,
`/dev/tty.usbmodem*` on macOS).

```
fantasi> help
  batt        show battery level
  ble         BLE status [list|on|off]
  cat         print file contents
  cd          change directory
  cdc         return USB to serial (CDC) mode
  connect     connect to a BLE peer
  crc32       CRC32 of a device file
  device      print device identifier
  df          show free flash bytes
  dfu         reboot into bootloader/DFU
  disconnect  disconnect BLE peer(s)
  echo        print arguments back
  exit        exit the CLI
  free        report heap and SRAM free bytes
  help        list commands
  kill        stop the running app: kill [pid]
  launch      load and run an app (/ramfs or /apps path)
  log         stream or write log messages
  ls          list files
  memtest     write/read-back test free SRAM
  mkdir       create a directory
  msc         enter USB mass storage mode
  pair        pair with a BLE device
  ps          list tasks (state, priority, free stack) + heap summary
  pwd         print working directory
  quit        exit the CLI
  radio       show radio stack info
  reboot      restart the device
  rm          delete a file
  rmdir       remove empty directory
  scan        scan for BLE devices
  settings    manage saved settings
  shutdown    power the device off
  unpair      remove BLE bond(s)
  upload      copy host file to device
  uptime      show time since boot
  version     show firmware version
  webusb      switch USB to WebUSB (protobuf) mode
  whoami      show unique device name
```
