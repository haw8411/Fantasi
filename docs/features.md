# Feature matrix

| Feature | Flipper Zero | Kiisu | Chameleon Ultra | Proxmark3 |
|---|---|---|---|---|
| **MCU** | STM32WB55 (Cortex-M4) | STM32WB55 (Cortex-M4) | nRF52840 (Cortex-M4) | AT91SAM7S512 (ARM7TDMI) |
| **Flash** | 1 MB | 1 MB | 1 MB | 512 KB |
| **RAM** | 192 KB | 192 KB | 213 KB | 64 KB |
| Serial CLI (USB CDC) | Yes | Yes | Yes | Yes |
| DFU / bootloader reboot | Yes | Yes | Yes | Yes |
| Battery level | Yes (BQ27220 fuel gauge) | Yes (BQ27220 fuel gauge) | Yes (SAADC voltage sense) | No |
| Display | Yes (ST7565 128x64 LCD) | Yes (SH1106 via STM32G431 companion MCU) | No | No |
| BLE full extended stack | Yes (STM32WB CPU2) | Yes (STM32WB CPU2) | Yes (SoftDevice S140) | No |
| BLE scanning | Yes | Yes | Yes | No |
| LittleFS storage (256 KB) | Yes (internal flash) | Yes (internal flash) | Yes (internal flash) | Yes (flash plane 1, S512 only) |
| USB MSC (mass storage) | Concurrent with CDC | Concurrent with CDC | Concurrent with CDC | Mode-switched (`msc` command) |
| MSC return to CDC | N/A (always concurrent) | N/A (always concurrent) | N/A (always concurrent) | `eject /dev/sdX` from host |

## Notes

- **Proxmark3 MSC mode switching**: The AT91SAM7S UDP has only 4 endpoints, all used by CDC. MSC requires a full USB re-enumeration - the `msc` CLI command disconnects CDC, re-enumerates as MSC-only, and `eject` on the host switches back to CDC automatically.
- **Proxmark3 S256 vs S512**: LittleFS storage requires the AT91SAM7S512 variant (two flash planes). The S256 variant has no room for storage; `msc` reports "storage not available."
- **BLE scanning on Flipper Zero and Kiisu**: Requires a BLE wireless stack flashed to CPU2 (both are STM32WB55). See [radio_stack.md](radio_stack.md) for stack installation.
- **Display**: The Flipper Zero and Kiisu both have a screen. The `display_print()` API is platform-local under `platforms/flipper/` (reused by `platforms/kiisu/`). The Flipper drives its ST7565 directly, while the Kiisu drives an SH1106 through a companion STM32G431 MCU (built with `-DFANTASI_DISPLAY_COMPANION`).
