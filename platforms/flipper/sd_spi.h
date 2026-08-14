/* Flipper Zero microSD - SD-over-SPI (bare-metal, no ST HAL).
 *
 * The card sits on SPI2 (the Flipper's "bus D"), shared with the ST7565 display
 * (platforms/flipper/display.c). Both run SPI mode 0; the SD reconfigures the
 * clock and drives its own chip-select (PC12) per burst, while the display has
 * no MISO and its own CS. Concurrent access to the shared bus must be serialised
 * (a display/SD SPI2 mutex - added once the raw driver is proven).
 *
 * Pins: SCK=PD1, MOSI=PB15, MISO=PC2 (all AF5), CS=PC12 (GPIO). 512-byte sectors.
 */
#ifndef FLIPPER_SD_SPI_H
#define FLIPPER_SD_SPI_H

#include <stdint.h>
#include <stdbool.h>

/* Bring the card up: power-up clocks, CMD0/CMD8/ACMD41/CMD58 init handshake,
 * then switch to the fast clock. Idempotent-ish: safe to retry. Returns true if
 * a card was found and initialised. */
bool sd_spi_init(void);

/* True once sd_spi_init() has succeeded. */
bool sd_spi_ready(void);

/* Diagnostics: how far the last sd_spi_init() got, and the R1 byte of the step
 * that stalled it. Stage codes:
 *   0 CMD0 never idled (no card / no clocks)   1 CMD8 done (voltage check)
 *   2 ACMD41 ready                              3 CMD58/OCR read
 *   4 block length set                          5 fully initialised
 * card_type: 0 none, 1 SD v1/MMC, 2 SD v2 (SDHC/SDXC). */
int     sd_spi_diag_stage(void);
uint8_t sd_spi_diag_r1(void);
int     sd_spi_diag_card_type(void);

/* Total addressable 512-byte sectors (from the CSD), 0 if unknown. */
uint32_t sd_spi_sector_count(void);

/* Read/write `count` consecutive 512-byte sectors starting at `lba`.
 * Return 0 on success, -1 on error. */
int sd_spi_read(uint32_t lba, uint8_t *buf, uint32_t count);
int sd_spi_write(uint32_t lba, const uint8_t *buf, uint32_t count);

#endif /* FLIPPER_SD_SPI_H */
