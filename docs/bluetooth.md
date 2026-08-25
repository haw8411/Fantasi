# Bluetooth

Fantasi uses BLE for wireless CLI access and device-to-device pairing. All BLE
traffic uses nanopb protobuf framing (CliRequest/CliResponse) over a GATT serial
service.

Each host CLI process opens its own device session; processes do not coordinate
or share a host-side connection owner. BlueZ may share one physical BLE link,
but notifications carry a device session ID and are filtered independently.
Large requests use per-ATT-write envelopes (`session`, total length, offset), so
fragments from simultaneous CLI processes can interleave safely. Ctrl-C cancels
that session's active command. The `w` command shows all live BLE/WebUSB sessions.

## Quick start

The host CLI pairs on demand - you don't need `bluetoothctl`. Point it at an
address and, if not already bonded, it pairs first (registering a `KeyboardOnly`
agent), then connects:

```bash
# 1. Find the address (host scan, or `scan` on another Fantasi)
bluetoothctl scan on        # look for "Fantasi <name>"

# 2. Connect - pairs automatically on first use
build/cli/fantasi --ble-addr=<addr>
# Enter the 6-digit passkey shown in the device's USB `log` when prompted
fantasi> whoami
Sur1d0r
fantasi> ls /
  settings.cfg          6
  splash.bin            1024
fantasi> upload local_file.bin /remote_path.bin
  1024 / 1024 bytes
```

To pair manually with `bluetoothctl` instead, see [Pairing](#pairing) below.

## Supported targets

| Target | BLE Stack | Peripheral | Central | Pairing |
|---|---|---|---|---|
| Flipper Zero | STM32WB (CPU2) | ✓ | ✓ | Passkey Entry (KEYBOARD_ONLY central, DISPLAY_ONLY peripheral) |
| Kiisu | STM32WB (CPU2) | ✓ | ✓ | Passkey Entry (KEYBOARD_ONLY central, DISPLAY_ONLY peripheral) |
| Chameleon Ultra | Nordic S140 v7.2.0 | ✓ | ✓ | Passkey Entry (KEYBOARD_ONLY central, DISPLAY_ONLY peripheral) |
| Proxmark3 | - | - | - | - |
| Proxmark5 | ESP32-C2 | - | - | - |

## Service UUIDs

| Target | Service UUID |
|---|---|
| Flipper Zero | `8fe5b3d5-2e7f-4a98-2a48-7acc60fe0000` |
| Kiisu | `8fe5b3d5-2e7f-4a98-2a48-7acc60fe0000` |
| Chameleon Ultra | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` (Nordic UART Service) |

The host CLI auto-detects either UUID set when connecting.

## Pairing

All pairing uses Passkey Entry with MITM protection. One side displays a 6-digit
code, the other side enters it. The passkey is logged via the `log` command on
the displaying device.

### Host → Flipper Zero (peripheral)

The Flipper advertises as `Fantasi <name>` and uses `DISPLAY_ONLY`. BlueZ with a
`KeyboardOnly` agent negotiates Passkey Entry: the Flipper generates and displays
a 6-digit code on the OLED + log, and the host must enter it. There is no
Numeric-Comparison / auto-confirm path - a peer cannot bond without the code.

```
# On the host
bluetoothctl
  agent KeyboardOnly
  default-agent
  scan on
  pair <flipper_addr>
  # Enter the 6-digit code shown on the Flipper OLED / `log`
```

### Host → Chameleon Ultra (peripheral)

The CU advertises as `Fantasi <name>` and uses `DISPLAY_ONLY`. BlueZ with a
`KeyboardOnly` agent negotiates Passkey Entry. The CU displays the code in the
log; the host enters it.

```
# On the CU, start log streaming via USB serial
fantasi> log

# On the host
bluetoothctl
  agent KeyboardOnly
  default-agent
  scan on
  pair <cu_addr>
  # Read the 6-digit code from the CU log stream
  # Enter it when prompted
```

### Flipper Zero (central) → Chameleon Ultra (peripheral)

The Flipper connects as central with `KEYBOARD_ONLY` IO cap. The CU displays
the passkey in its log. The Flipper prompts `enter passkey:` on the serial CLI.

```
# On the CU, start log streaming via USB serial
fantasi> log

# On the Flipper
fantasi> pair <cu_ble_addr> r
  # "r" flag = random address (CU uses a random BLE address)
  # Wait for "enter passkey:" prompt
  # Read the 6-digit code from the CU log stream
  # Type it and press Enter
```

### Chameleon Ultra (central) → Flipper Zero (peripheral)

The CU connects as central with `KEYBOARD_ONLY` IO cap. The Flipper displays
the passkey on its OLED and in its log. The CU prompts `enter passkey:`.

```
# On the Flipper, start log streaming via USB serial (optional, OLED also shows it)
fantasi> log

# On the CU
fantasi> pair <flipper_ble_addr>
  # Wait for "enter passkey:" prompt
  # Read the 6-digit code from the Flipper's OLED or log
  # Type it and press Enter
```

### Flipper Zero (central) → Flipper Zero (peripheral)

Same as FZ → CU, but the peripheral Flipper auto-handles the incoming pair
request via `ble_serial_poll` - it generates a passkey, shows it on the OLED,
and responds to the stack automatically. The central Flipper prompts for the
passkey.

```
# On the peripheral Flipper (target), just leave it running - it auto-advertises
# The passkey will appear on the OLED display and in the log

# On the central Flipper
fantasi> pair <target_ble_addr>
  # Read the passkey from the target's OLED
  # Type it when prompted
```

### Chameleon Ultra (central) → Chameleon Ultra (peripheral)

Same as CU → FZ. The peripheral CU logs the passkey via `fantasi_log`. The
central CU prompts for it.

```
# On the peripheral CU
fantasi> log

# On the central CU
fantasi> pair <target_ble_addr> r
  # Read the passkey from the peripheral's log stream
  # Type it when prompted
```

## Host CLI BLE connection

After pairing, the host CLI connects over BLE using the `--ble` flag. With
multiple Fantasi devices in range, use `--ble-addr=` to target a specific one.

```
fantasi --ble
fantasi --ble-addr=F8:F5:60:D9:51:CE
```

All commands work over BLE including file operations (`ls`, `cat`, `upload`,
`rm`, `mkdir`).

## Troubleshooting

### `pair failed: Authentication Timeout` (no passkey prompt)

The device persists its bond across reboots. If the host has forgotten the bond
(or you cleared it) but the device still holds the old key, a fresh pair attempt
stalls - BlueZ reports `Authentication Timeout` and the host never prompts for a
passkey. This is an **asymmetric bond**: clear it on the **device** side, since
the host can't reach the device's bond over BLE.

```
# Clear the device's bond over USB serial
fantasi /dev/ttyACM*      # then:  unpair

# (Optional) clear any host-side cache entry - usually already clean
bluetoothctl remove <addr>

# Retry - you should now get the passkey prompt
fantasi --ble
```

The host CLI prints this hint automatically when pairing fails this way.

## Finding BLE addresses

Use the `scan` command on any Fantasi device to discover nearby devices:

```
fantasi> scan
F8:F5:60:D9:51:CE  -59 dBm  "Fantasi Thadusag"
C2:50:09:7F:30:7B  -51 dBm  "Fantasi Sur1d0r"
```

Or from the host:

```
bluetoothctl scan on
```

## Logging

The `log` command streams timestamped log entries. BLE pairing codes appear
here when the device is acting as a peripheral (displaying the passkey).

```
fantasi> log
[0000000.000] [I] boot
[0000054.407] [I] BLE pair code: 588350
[0000055.123] [I] BLE pair ok (0x00)
```

Press Ctrl-C to exit the log stream.

The `log write <message>` subcommand adds a custom entry:

```
fantasi> log write hello
fantasi> log
[0000000.000] [I] boot
[0000012.345] [I] hello
```
