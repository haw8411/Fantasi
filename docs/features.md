# Feature matrix

| Feature | Flipper Zero | Kiisu | Chameleon Ultra | Proxmark3 | Proxmark5 |
|---|---|---|---|---|---|
| **MCU** | STM32WB55 (Cortex-M4) | STM32WB55 (Cortex-M4) | nRF52840 (Cortex-M4) | AT91SAM7S512 (ARM7TDMI) | AT32F435 (Cortex-M4) |
| **Flash** | 1 MB | 1 MB | 1 MB | 512 KB | 1 MB (dual-bank) |
| **RAM** | 192 KB | 192 KB | 213 KB | 64 KB | 384 KB |
| Serial CLI (USB CDC) | Yes | Yes | Yes | Yes | Yes |
| DFU / bootloader reboot | Yes | Yes | Yes | Yes | Yes (AT32 ROM DFU) |
| Battery level | Yes (BQ27220 fuel gauge) | Yes (BQ27220 fuel gauge) | Yes (SAADC voltage sense) | No | No (in progress) |
| Display | Yes (ST7565 128x64 LCD) | Yes (SH1106 via STM32G431 companion MCU) | No | No | No |
| BLE full extended stack | Yes (STM32WB CPU2) | Yes (STM32WB CPU2) | Yes (SoftDevice S140) | No | No (in progress) |
| BLE scanning | Yes | Yes | Yes | No | No |
| LittleFS storage (256 KB) | Yes (internal flash) | Yes (internal flash) | Yes (internal flash) | Yes (flash plane 1, S512 only) | Yes (flash bank 2) |
| USB MSC (mass storage) | Concurrent with CDC | Concurrent with CDC | Concurrent with CDC | Mode-switched (`msc` command) | Concurrent with CDC |
| MSC return to CDC | N/A (always concurrent) | N/A (always concurrent) | N/A (always concurrent) | `eject /dev/sdX` from host | N/A (always concurrent) |
