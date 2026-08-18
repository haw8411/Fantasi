# Fantasi CLI

The Fantasi CLI is a host-side tool that wraps the device's serial interface and MSC storage into a single interactive shell. It connects to a Fantasi device over USB **or BLE**, forwarding commands to the firmware while also providing local file-management commands that operate on the device's storage (over the MSC `Fantasi` FAT volume on USB, or via protobuf framing over BLE).

## Building

A bare `make` (no `PLATFORM`) builds the host CLI alongside the firmwares. To build just the host tool:

```
make cli        # or: make -C cli
```

The binary is placed at `build/cli/fantasi`. It has no filesystem dependency of its own: device storage is exposed by the firmware as a synthetic FAT volume labelled `Fantasi`, which the host OS mounts like any USB stick - the CLI does plain file I/O against that mountpoint, so there is no on-disk-format coupling. Requires `libreadline-dev` for line editing and history, plus `udisksctl` (udisks2) and `findmnt` (util-linux) to mount the FAT. BLE transport is compiled in only when `libsystemd` (BlueZ over D-Bus) is available at build time; without it the `--ble` flags are unavailable.

## Usage

```
fantasi [--ble[=ADDR]] [--name NAME] [/dev/ttyACMx] [/dev/sdX]
```

All arguments are optional:

- No arguments - auto-detect a Fantasi device over USB (VID `1209` / PID `0001`, resolving the CDC and MSC interfaces). If no USB device is found, it falls back to BLE.
- `/dev/ttyACMx` and/or `/dev/sdX` - pin a specific CDC port and/or MSC block device.
- `--name NAME` - select a specific device by its name (the name `whoami` prints) when several Fantasi devices are connected. The name is the device's USB serial descriptor and BLE advertised name (`Fantasi <NAME>`), so it disambiguates over USB and BLE alike.
- `--ble` - connect over BLE to the first paired/available Fantasi.
- `--ble-addr=AA:BB:CC:DD:EE:FF` - connect over BLE to a specific address.

Over BLE the CLI pairs on demand (it registers a BlueZ `KeyboardOnly` agent before connecting) - enter the passkey shown in the device's USB `log`. See [bluetooth.md](bluetooth.md) for pairing details.

The FAT volume is mounted through `udisksctl`, which runs unprivileged for the logged-in user, so the CLI normally needs no `sudo`. BLE access does not touch storage at all.

## Commands

### All commands

Type `help` to see every command. Local commands (run on the host, operate on the device's storage via MSC) are shown in **yellow**. Firmware commands (sent to the device over serial) are shown in white.

Local commands:

| Command | Description |
|---|---|
| `cat <file>` | Print file contents |
| `cd <path>` | Change directory |
| `clear` | Clear the terminal screen |
| `cp <src> <dst>` | Copy a file |
| `crc32 <file>` | Print the CRC32 (and size) of a device file |
| `exit` / `quit` | Exit the CLI |
| `ls [path]` | List files |
| `mkdir <dir>` | Create a directory |
| `mv <src> <dst>` | Move/rename a file |
| `pwd` | Print working directory |
| `rm <file>` | Delete a file |
| `rmdir <dir>` | Remove an empty directory |
| `upload <local> <remote>` | Copy a host file to the device filesystem |

Paths are resolved relative to the current directory. Use `/` for absolute paths.

Everything else is forwarded to the device over serial - type any firmware command directly:

```
fantasi> scan 3
fantasi> ble off
fantasi> settings
fantasi> whoami
```

## Examples

### Upload a splash screen

```
$ build/cli/fantasi
serial: /dev/ttyACM0
storage: on-demand (FAT auto-mount)
fantasi> upload art/fantasi_splash.bin /splash.bin
art/fantasi_splash.bin -> /splash.bin (1024 bytes)
fantasi> ls
  splash.bin           1024
  settings.cfg         6
```

### Toggle BLE and check settings

```
fantasi> ble off
ble off
fantasi> settings
ble=0
fantasi> ble on
ble on
fantasi> radio
secure flash: 236 KB @ 0x080C5000
BLE:   on
```

### Check device identity

```
fantasi> whoami
Shetak0
fantasi> device
FZ
```

## How it works

Device storage is presented over USB MSC as a single synthetic FAT volume (label `Fantasi`). The firmware synthesizes the FAT boot sector, FAT tables, and directory entries on the fly from its real filesystems - internal flash (LittleFS, mounted at `/`) and, on app-capable targets, the RAM-backed `/ramfs` - so there is no second copy of the data and no fixed image size. Reads are served from those filesystems; writes are parsed back out of the FAT directory/data sectors and committed to the underlying filesystem.

The first local command (`ls`, `upload`, …) triggers the CLI to mount that volume. On composite devices (Flipper, Kiisu, Chameleon, Proxmark5) the block device is always present alongside CDC; on switch-mode devices (Proxmark3, which reuses its CDC endpoints for MSC) the CLI first sends the `msc` command to flip the device into MSC mode. Either way the volume is then mounted with `udisksctl`, and local commands are plain stdio against the mountpoint. When the CLI next needs the serial port (a forwarded firmware command, or exit) it unmounts; on switch-mode devices it also SCSI-ejects so the firmware re-enumerates as CDC.

Serial commands are sent as raw text over the CDC port. The CLI strips echo and prompt lines from the response before printing.

## Storage notes

- Because the FAT is synthetic, the host sees a normal removable drive - you can also mount and browse it with your file manager. Long (non-8.3) filenames are supported via VFAT LFN entries.
- Composite devices (FZ, Kiisu, CU, PM5) keep CDC and MSC active simultaneously, so the CLI freely interleaves serial commands and storage access (unmounting only momentarily to free the volume for the firmware).
- Switch-mode devices (PM3) trade the CDC endpoints for MSC, so each storage operation is bracketed by a mode switch in and a SCSI eject out; the CLI handles this transparently and waits for the CDC port to reappear.
- `crc32 <file>` reads a device file and prints its CRC32 and size - used by `tools/flash.py` to skip re-uploading unchanged resources without capturing binary data over the link.
