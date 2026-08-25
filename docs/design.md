# Fantasi technical design

Fantasi is FreeRTOS firmware for the Flipper Zero, Kiisu, Chameleon Ultra,
Proxmark3, and Proxmark5. Most of the firmware is shared. Each board supplies
startup code and drivers through the hardware abstraction layer (HAL).

## Architecture

```text
┌──────────────────────────────────────────────────────┐
│ Applications                                         │
├──────────────────────────────────────────────────────┤
│ Fantasi core                                         │
│ CLI · sessions · VFS · app loader · shared services  │
├──────────────────────────────────────────────────────┤
│ Hardware abstraction layer                           │
├──────────────────────────────────────────────────────┤
│ Platform implementations                             │
│ startup · MCU and peripheral drivers                 │
├──────────────────────────────────────────────────────┤
│ FreeRTOS                                             │
├──────────────────────────────────────────────────────┤
│ Hardware                                             │
└──────────────────────────────────────────────────────┘
```

`core/` contains the command dispatcher, protobuf session engine, VFS, app
loader, Berry host, logging, power policy, and shared RFID code. `hal/` declares
the hardware interfaces and contains USB and storage code used by several
boards. `platforms/` implements the HAL and contains each target's startup code,
linker script, and drivers.

Build flags leave out subsystems that a board cannot use. HAL capability values
handle differences that must be checked at runtime, such as the presence of a
display.

## Runtime

CDC provides one line-oriented shell. WebUSB and BLE carry protobuf requests;
each host process gets a logical session. Requests run in order within a
session, while active sessions can run concurrently. Commands use the same
dispatcher on every transport. Protobuf file requests call the VFS directly.

The WebUSB and BLE ingress tasks collect and route input. A worker exists
while its session has work queued, then exits. Native apps run on their own
task. Hardware and storage are shared across sessions, and the firmware runs at
most one native app at a time.

## Storage

The VFS maps absolute paths to three kinds of storage:

| Path | Backend |
|---|---|
| `/` | Internal-flash LittleFS |
| `/ramfs` | Heap-backed temporary files |
| `/mnt/extN` | External LittleFS or FatFs volumes |

Commands, apps, scripts, and host file requests use these paths. USB mass
storage presents the same tree as a generated FAT volume and translates host
access back into VFS operations.

The Flipper Zero, Kiisu, Chameleon Ultra, and Proxmark5 expose CDC and mass
storage together. The Proxmark3 has too few USB endpoints for that arrangement,
so it switches between CDC and mass storage.

## Applications

A native app is a relocatable ELF stored in the VFS, usually in `/ramfs` or
`/apps`. The loader copies its allocated sections into RAM, applies relocations,
resolves the app API, and starts `app_main`. The API covers console I/O, memory,
files, timing, and available hardware. App memory and peripheral state are
released when the app exits or is stopped.

App source is portable, but binaries match the CPU architecture. The Flipper Zero,
Kiisu, Chameleon Ultra, and Proxmark5 use the Cortex-M4 build; the Proxmark3 uses
the ARM7 build. Berry scripts run in a VM connected to the VFS and hardware API.

See [apps.md](apps.md) for the app ABI and [rfid.md](rfid.md) for the RFID app.

## Source layout

```text
apps/       App API and loadable applications
cli/        Host CLI and its USB/BLE transports
core/       Shared firmware runtime
hal/        Hardware interfaces and shared device code
platforms/  Board startup, linker configuration, and drivers
proto/      Protobuf schema and generated messages
```

Start with `core/main.c` for boot and task creation, `core/cli.c` for commands,
`core/proto.c` for WebUSB and BLE sessions, `core/vfs.c` for storage routing,
and `core/app_run.c` for app execution. Platform work starts at `hal/hal.h` and
the nearest existing board under `platforms/`.
