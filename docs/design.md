# Fantasi technical design

Fantasi is a multi-platform firmware that runs on security research devices - the Flipper Zero, Kiisu, Chameleon Ultra, and Proxmark3. It replaces each device's stock firmware with a common runtime that presents a unified USB CLI and storage interface, then loads and executes applications transferred from the host.

## Architecture

Fantasi is a FreeRTOS-based system. The kernel provides preemptive multitasking, heap management, and synchronization primitives. On top of it, a small HAL layer abstracts the hardware differences between targets so that core logic - the CLI, storage, and (eventually) the application loader - is written once and compiled for all platforms.

```
  ┌──────────────────────────────────┐
  │         Applications             │  ← loaded into RAM from host
  ├──────────────────────────────────┤
  │    core/       CLI, commands     │
  │    hal/        serial, storage   │
  ├──────────────────────────────────┤
  │  platform HAL implementations    │
  ├──────────────────────────────────┤
  │          FreeRTOS kernel         │
  ├──────────────────────────────────┤
  │          MCU hardware            │
  └──────────────────────────────────┘
```

### Tasks

| Task | Priority | Role |
|---|---|---|
| USB | `tskIDLE + 2` | Drives TinyUSB (`tud_task`), services CDC and MSC class drivers |
| CLI | `tskIDLE + 1` | Line-editing, command dispatch, serial I/O |
| BLE CLI (`blecli`) | `tskIDLE + 1` | BLE-only (FZ/Kiisu/CU): pumps the BLE stack, runs the protobuf CLI transport and the LittleFS write path for BLE file ops |
| Idle | `tskIDLE` | FreeRTOS idle hook (unused) |

The `blecli` task is created only on platforms built with `FANTASI_ENABLE_BLE_CLI` (Flipper Zero, Kiisu, and Chameleon Ultra); the Proxmark3 has no radio and omits it.

### Code structure

```
core/           Shared across all targets
  main.c          Task creation, scheduler start
  cli.c           Line editor, command dispatch loop
  commands.c      Built-in commands (help, free, scan, msc, ...)

hal/            Hardware abstraction
  hal.h           HAL contract - every platform implements these
  storage/        LittleFS mount/unmount, MSC block-device callbacks
  tinyusb/        Shared USB descriptors, CDC serial transport

platforms/      One directory per target
  flipper/        STM32WB55 - hal.c, display, BLE, flash driver, linker
  kiisu/          STM32WB55 - Flipper-compatible board; reuses flipper/ with a
                  companion-MCU display
  chameleon/      nRF52840  - hal.c, BLE, flash driver, linker
  proxmark3/      AT91SAM7S - hal.c, flash driver, USB mode-switch, linker

third_party/    Auto-cloned dependencies (gitignored)
  tinyusb/        USB device stack (pinned tag)
  littlefs/       On-flash filesystem (pinned tag)
  nanopb/         Protobuf codec for the BLE CLI transport
  FreeRTOS-Kernel/
  cmsis_core/
  stm32wb_cmsis/
  nrf52_mdk/
```

### HAL contract

Every platform implements the functions declared in `hal/hal.h`:

- `hal_init` - clock, GPIO, peripheral setup
- `hal_serial_read/write/connected` - USB CDC transport
- `hal_reboot`, `hal_reboot_dfu` - warm reset, bootloader entry
- `hal_battery_percent` - fuel gauge or ADC reading (-1 if unavailable)
- `hal_ble_scan` - passive BLE scan with per-device callback (-1 if no radio)
- `hal_flash_free_bytes` - free flash between firmware end and reserved regions
- `hal_enter_msc_mode` - USB mode switch to MSC (PM3), or -1 if MSC is concurrent (FZ/Kiisu/CU)

Platforms that lack a feature return a sentinel value rather than requiring compile-time exclusion. The CLI adapts at runtime.

## USB

All targets use TinyUSB as the USB device stack. The Flipper Zero, Kiisu, and Chameleon Ultra run CDC and MSC as a composite device (both interfaces active simultaneously). The Proxmark3 has only 4 hardware endpoints, so it mode-switches between CDC and MSC on demand - the `msc` CLI command re-enumerates as MSC-only, and host-side `eject` returns to CDC.

## Storage

Each device reserves 256 KB of internal flash for a LittleFS filesystem, placed at the top of available flash so firmware updates (which write from the bottom) never touch stored files.

| Target | Storage region | Placement logic |
|---|---|---|
| Flipper Zero | Just below BLE secure flash (SFSA) | Runtime-detected from `FLASH->SFR` |
| Kiisu | Just below BLE secure flash (SFSA) | Runtime-detected from `FLASH->SFR` (same as Flipper) |
| Chameleon Ultra | Just below DFU bootloader | Runtime-detected from `UICR.NRFFW[0]` |
| Proxmark3 | Flash plane 1 (`0x140000`) | Fixed; S512 only |

The on-device firmware mounts LittleFS at boot and reads/writes files via the `lfs_*` API. To the host it does *not* expose the raw LittleFS blocks; instead the MSC LUN presents a **synthetic FAT volume** (labelled `Fantasi`, 512-byte sectors) that is generated on the fly from the underlying filesystem(s). Reads synthesize the boot sector, FAT tables, and directory/data sectors from the live tree (LittleFS at `/`, plus the RAM-backed `/ramfs` on app-capable targets); writes are parsed back out of the FAT structures and committed to the real filesystem. This means any host OS can mount the volume like a normal USB stick - no LittleFS knowledge on the host side - and there is no second copy of the data in RAM. See [cli.md](cli.md) for how the host CLI drives this. The synthetic FAT is implemented in `hal/storage/fat_ramdisk.c` over the `core/vfs.c` path router.

## Future: application model

Fantasi's long-term role is a runtime and loader, not a monolithic firmware. The execution model:

1. **Host deploys an application** via the Fantasi web flasher or CLI tool. The binary is transferred over USB into device RAM.
2. **The loader executes the application** from RAM. The application has access to a defined API surface (serial I/O, storage, radio, display where available).
3. **If the application needs persistence**, it reads and writes files on the LittleFS volume. Configuration, calibration data, captured samples - anything that should survive a power cycle lives here.
4. **An optional startup executable** can be stored in LittleFS. On boot, if a startup binary is present, the loader runs it automatically instead of dropping to the interactive CLI. This lets a device behave as a dedicated tool (e.g., BadUSB, or a long-running BLE scanner) without a host connection.
5. **Applications are source-portable** across targets. The HAL and loader API abstract hardware differences, with runtime capability queries for target-specific features (display, BLE, etc.). Binaries are architecture-specific (the relocatable ELF is built per core - Cortex-M and ARM7TDMI), so the same source builds one loadable variant per target.

The first cut of this model is implemented today: `launch <path>` loads a relocatable ELF from `/ramfs` or `/apps` into RAM, runs it on a dedicated interruptible task, and frees it on exit or Ctrl-C. See [apps.md](apps.md).

The web flasher provides a browser-based drag-and-drop interface for deploying applications to any connected Fantasi device over Web Bluetooth. The host CLI tool (`build/cli/fantasi`, see [cli.md](cli.md)) provides the same functionality over USB or BLE for scripted and headless workflows.
