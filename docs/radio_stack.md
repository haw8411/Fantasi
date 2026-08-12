# Radio Stack Flashing

The Flipper Zero (STM32WB55), Kiisu (STM32WB55), and Chameleon Ultra (nRF52840)
have dedicated radio
coprocessors that run a pre-compiled BLE stack. The stack binary is encrypted and
signed by the chip vendor - it cannot be built from source. It must be installed
separately from the Fantasi application firmware.

The Proxmark3 has no radio coprocessor. The Proxmark5 has no on-die radio coprocessor
either; its BLE comes from an ESP32-C2 module (development in progress), so this
flashing procedure does not apply to it.

## Flipper Zero (STM32WB55)

The Kiisu uses the same STM32WB55 MCU and the same CPU2 wireless stack as the
Flipper Zero, so this entire section (FUS, binary selection, and the
`radio_flash.py` install procedure) applies to the Kiisu unchanged.

The STM32WB55 has two CPU cores:

- **CPU1** (Cortex-M4): runs Fantasi
- **CPU2** (Cortex-M0+): runs the BLE wireless stack

CPU2's firmware lives in a protected "secure flash" region managed by the **FUS**
(Firmware Upgrade Services) - a small ROM-like service on CPU2 that handles
installation, deletion, and verification of wireless stack binaries. FUS is the
only entity that can install the stack because the binaries are encrypted and
signed by ST.

### Prerequisites

- Python 3 with `pyusb`: `pip install pyusb`
- The device in DFU mode: `fantasi> dfu`

### Which binary?

The binary **must match the FUS version on your device**. The Flipper Zero ships
with FUS 1.2.0. By default `radio_flash.py` fetches the right one for you:
`stm32wb5x_BLE_Stack_full_extended_fw.bin`, pinned to
[`flipperdevices/stm32wb_copro`](https://github.com/flipperdevices/stm32wb_copro)
tag `v1.20.0`, verified against a known SHA-256 and cached under `build/copro/`
(so later runs are offline). You can override with an explicit path.

**Use the flipperdevices/stm32wb_copro binaries, not ST's STM32CubeWB ones** - the
STM32CubeWB builds are incompatible with the Flipper's FUS 1.2.0 and fail on-device
with `IMG_NOT_AUTHENTIC`. The SHA gate in `radio_flash.py` enforces this: it refuses
to flash anything but the pinned binary unless you pass a path yourself.

Variants (all in that repo; pass a path to use one other than the default):

| Binary | Size | Features |
|---|---|---|
| `stm32wb5x_BLE_Stack_light_fw.bin` | ~117 KB | Peripheral + GATT server, up to 4 links |
| `stm32wb5x_BLE_Stack_full_fw.bin` | ~152 KB | Central + peripheral, up to 8 links |
| `stm32wb5x_BLE_Stack_full_extended_fw.bin` | ~189 KB | Full + extended advertising/scanning (default) |

### Flashing

```bash
# 1. Enter DFU mode
fantasi> dfu

# 2. Run the flashing tool (no arg: fetch + SHA-verify the pinned default)
python3 tools/radio_flash.py

# ...or flash a specific binary you already have:
python3 tools/radio_flash.py path/to/stm32wb5x_BLE_Stack_full_extended_fw.bin
```

The tool:
1. Reads the current FUS state
2. Activates FUS (if the wireless stack is running, sends two GET_STATE commands
   to switch CPU2 from wireless-stack mode to FUS mode)
3. Deletes the old wireless stack (FUS_FW_DELETE)
4. Stages the new binary in user flash
5. Sends FUS_FW_UPGRADE - FUS reads the staged binary, verifies the ST
   signature, decrypts it, and installs it into the secure flash region
6. Sends FUS_START_WS to boot the new stack

The process takes 30-60 seconds. The device resets multiple times as FUS
reconfigures the secure flash boundaries. When complete, the device boots back
into Fantasi.

### Verifying

```
fantasi> radio
secure flash: 512 KB @ 0x08080000
```

The secure flash size reflects the installed stack:

| Stack | Typical secure flash |
|---|---|
| None (FUS only) | ~48 KB |
| BLE Light | ~164 KB |
| BLE Full Extended | ~512 KB |

### How it works

`radio_flash.py` communicates with FUS through the STM32 ROM DFU bootloader
using the protocol documented in ST's AN5185. FUS commands are sent as single-byte
DFU DNLOAD payloads:

| Command | Byte | Purpose |
|---|---|---|
| FUS_FW_DELETE | `0x52` | Delete installed wireless stack |
| FUS_FW_UPGRADE | `0x53` | Install staged binary |
| FUS_GET_STATE | read `0xFFFF0054` | Poll FUS state and error code |
| FUS_START_WS | `0x5A` | Boot the wireless stack |

This is the same protocol used by qFlipper and STM32CubeProgrammer.

### Recovery

If the flashing is interrupted (power loss, USB disconnect):

- **FUS is never at risk.** It lives in a protected region and cannot be erased
  by any of these operations. The ROM DFU bootloader is also in read-only system
  memory.
- **If the wireless stack was deleted but the new one wasn't installed:** enter DFU
  and run `radio_flash.py` again. FUS will be in idle state, ready to accept a new
  stack.
- **If the device doesn't boot:** enter DFU via the button combo (hold OK + BACK
  for 30 seconds) and reflash the Fantasi firmware with `dfu-util`.

### Notes

- An SWD debug probe (Black Magic Probe, ST-Link, etc.) connected to the Flipper's
  SWD pins **prevents CPU2 from booting**. Disconnect the probe before running
  `radio_flash.py` or using the `radio` CLI command.
- The `radio` CLI command reports secure flash size from the SFSA option byte.
  It does not communicate with CPU2 directly - the wireless stack secures SRAM2A
  which makes IPCC access from CPU1 unsafe without a full transport layer.

## Chameleon Ultra (nRF52840)

The nRF52840 uses Nordic's SoftDevice as its BLE stack. It occupies flash from
`0x0000` to `0x27000` and is already present from the stock ChameleonUltra
firmware. Fantasi's linker starts the application at `0x27000` to preserve it.

Radio stack management for the CU is not yet implemented in Fantasi. The existing
SoftDevice from the stock firmware continues to work.
