# Development environment dependencies

Everything needed to build, flash, test, and debug Fantasi across its
targets (Flipper Zero / STM32WB55, Kiisu / STM32WB55, Chameleon Ultra / nRF52840,
Proxmark3 / AT91SAM7S). Grouped by task - you only need the rows for what you're doing.

The **Verified** column is what's installed in the current development
environment (and known-good); the **Install** column is how to get it on a fresh
Debian/Ubuntu box. Versions other than the pinned ones below are usually fine.

## Building firmware

| Dependency | Required | Verified | Purpose / install |
|---|---|---|---|
| `arm-none-eabi-gcc` + binutils | yes | Debian `gcc-arm-none-eabi` **14.2.1** (15:14.2.rel1-1) | Cross-compiler for all four firmwares. The build links against **picolibc** (`--specs=picolibc.specs`), so install via `sudo apt install gcc-arm-none-eabi picolibc-arm-none-eabi`. The upstream [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) tarball does not ship `picolibc.specs` required to build the firmware - use the Debian packages. `make` checks for a working toolchain (`check-toolchain`). |
| `picolibc-arm-none-eabi` | yes | 1.8.10-2 | The bare-metal C library the firmware links (`--specs=picolibc.specs`): float printf, ctype, string/mem, plus the malloc symbols we wrap onto the FreeRTOS heap. `sudo apt install picolibc-arm-none-eabi` |
| GNU `make` | yes | 4.4.1 | Top-level + per-platform build driver. `sudo apt install make` |
| `git` | yes | 2.47.2 | `make` auto-clones TinyUSB + LittleFS on demand (see below). `sudo apt install git` |

### Embedded third-party libraries (`third_party/`)

Pinned and reproducible. Two are fetched on first build; the rest are vendored
in-tree.

| Library | Version | How it's obtained | Used by |
|---|---|---|---|
| TinyUSB | **0.17.0** (`TUSB_TAG`) | `git clone` on demand (`check-tinyusb`) | Flipper, Kiisu, Chameleon (USB stack). Not Proxmark3. |
| LittleFS | **v2.11.3** (`LFS_TAG`) | `git clone` on demand (`check-littlefs`) | all targets (internal flash FS) |
| FreeRTOS-Kernel | vendored | in `third_party/` | all targets |
| nanopb (runtime) | vendored | in `third_party/` | protobuf encode/decode on device |
| CMSIS core | vendored | in `third_party/` | Cortex-M targets |
| nRF52 MDK | vendored | in `third_party/` | Chameleon |
| STM32WB CMSIS | vendored | in `third_party/` | Flipper, Kiisu |

To bump TinyUSB/LittleFS, edit `TUSB_TAG`/`LFS_TAG` in the top-level `Makefile`
(don't track a moving branch - it breaks reproducibility).

## Building the host CLI (`build/cli/fantasi`)

Built with the host compiler, not the cross-toolchain (`make cli`).

| Dependency | Required | Verified | Purpose / install |
|---|---|---|---|
| host `cc`/`gcc` | yes | Debian gcc 14.2.0 | compiles the CLI. `sudo apt install build-essential` |
| `libreadline-dev` | yes | installed | line editing + history. `sudo apt install libreadline-dev` |
| `pkg-config` | for BLE | - | detects libsystemd at build time. `sudo apt install pkg-config` |
| `libsystemd-dev` | optional | installed | enables the BLE transport (`--ble`) via BlueZ over D-Bus. Without it the CLI still builds; the `--ble` flags are just unavailable. `sudo apt install libsystemd-dev` |

## Host-side storage access (MSC)

The CLI exposes device storage as a synthetic FAT volume and drives it with
ordinary file I/O against an OS mount.

| Dependency | Required | Verified | Purpose / install |
|---|---|---|---|
| `udisks2` (`udisksctl`) | yes | installed | mounts the FAT volume without root. `sudo apt install udisks2` |
| `util-linux` (`findmnt`) | yes | 2.41 | resolves the mountpoint. (base system) |

## Flashing

See [flashing.md](flashing.md) for the full per-target procedure and recovery.
`make flash` (auto-detect) / `make PLATFORM=<x> flash` orchestrate these via
`tools/flash.py`.

| Target | Tool | Required | Verified | Install |
|---|---|---|---|---|
| Flipper (STM32WB55) | `dfu-util` | yes | 0.11 | `sudo apt install dfu-util` |
| Kiisu (STM32WB55) | `dfu-util` | yes | 0.11 | `sudo apt install dfu-util` |
| Chameleon (nRF52840) | `nrfutil` + `device` + `nrf5sdk-tools` commands | yes | device 2.13.2, nrf5sdk-tools 1.1.0 | install nrfutil, then `nrfutil install device nrf5sdk-tools` (used for signed `pkg generate` / `dfu` / `settings`) |
| Proxmark3 (AT91SAM7S) | `tools/pm3_flasher.py` → **pyserial** | yes | pyserial 3.5 | `pip install pyserial` |

## Debugging (SWD / JTAG)

Optional - only for low-level bring-up. See [flashing.md](flashing.md) and
[design.md](design.md).

| Dependency | Purpose | Verified | Install |
|---|---|---|---|
| `pyocd` | SWD over CMSIS-DAP (e.g. a Flipper acting as a DAP, or a DAP-Link probe); non-halting reads, RAM reset, breakpoints | 0.43.1 | `pip install pyocd` |
| Black Magic Probe + `gdb` | alternative SWD path (FZ); **do not** use it to flash here - only the flashing scripts flash | - | vendor probe + `gdb-multiarch` |
| `tools/radio_flash.py` → pyserial | flash an STM32WB wireless (BLE) stack via ROM DFU | pyserial 3.5 | `pip install pyserial` |

## BLE (host side)

Needed for the `--ble` CLI transport and the BLE integration tests.

| Dependency | Required | Verified | Purpose / install |
|---|---|---|---|
| BlueZ (`bluetoothctl`) | for BLE | 5.82 | host Bluetooth stack; tests pair/scan via it. `sudo apt install bluez` |
| D-Bus + `libsystemd` | for `--ble` | installed | CLI BLE transport (see host-CLI table) |
| `pexpect` (Python) | BLE tests | 4.9.0 | drives `bluetoothctl` passkey pairing. `pip install pexpect` |
| a Bluetooth adapter | for BLE | - | any BlueZ-supported BT 4.0+ adapter |

## Tests

- **Unit tests** (`make test-unit`, what CI runs): hardware-free - only Python 3.
- **Integration tests** (`make PLATFORM=<x> test`): need a connected device plus
  the relevant flashing/storage/BLE deps above. `pyserial` is required for the
  serial-command helpers; `pyusb` for the WebUSB vendor-pipe helpers (returning a
  switch-mode PM3 to CDC, and the cross-channel `ps_kill` test); BLE tests
  additionally need BlueZ + `pexpect`.

## Protobuf regeneration (`make proto`)

Only when editing `proto/fantasi.proto` (the committed `*.pb.{c,h}` are what
builds compile).

| Dependency | Verified | Install |
|---|---|---|
| `nanopb_generator` | nanopb 0.4.9.1 | `pip install nanopb` (override path with `make proto NANOPB=/path/to/nanopb_generator`) |

## Python

| Dependency | Required | Verified | Notes |
|---|---|---|---|
| Python 3 | yes | 3.13.5 | runs `tools/flash.py`, `tools/launch.py`, the test runners. 3.9+ expected. |
| `pyserial` | flashing/tests | 3.5 | PM3/radio flashers + integration serial helpers |
| `pyusb` | tests | 1.3.1 | WebUSB vendor-pipe helpers (switch-mode CDC recovery, cross-channel `ps_kill`); also `tools/radio_flash.py` |
| `pexpect` | BLE tests | 4.9.0 | `bluetoothctl` automation |
| `nanopb` | proto only | 0.4.9.1 | code generator (above) |

The launch helper (`tools/launch.py`, used by `make launch APP=<name>`) relies
only on the Python standard library (`pty`, `termios`, `select`).

## One-shot install (Debian/Ubuntu)

System packages:

```sh
sudo apt install build-essential make git \
                 gcc-arm-none-eabi picolibc-arm-none-eabi \
                 libreadline-dev libsystemd-dev pkg-config \
                 udisks2 util-linux \
                 dfu-util bluez
```

Python packages (a venv or `--user` is fine):

```sh
pip install pyserial pyusb pexpect nanopb pyocd
```

`nrfutil` (Chameleon flashing) is a separate vendor binary - install it, then:

```sh
nrfutil install device nrf5sdk-tools
```

> Note: the firmware links against **picolibc** (`--specs=picolibc.specs`), which
> the Debian `gcc-arm-none-eabi` package supplies alongside `picolibc-arm-none-eabi`.
> The upstream Arm GNU Toolchain tarball does not bundle `picolibc.specs`, so it is
> recommended to use the Debian packages.
