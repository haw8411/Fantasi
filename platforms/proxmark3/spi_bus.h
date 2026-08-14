/* Arbiter for the AT91SAM7 hardware SPI0 bus, which the PM3 shares between the
 * FPGA config register (NPCS0, driven by rfid.c) and the onboard SPI NOR flash
 * (NPCS2, driven by spi_flash.c). The two users have different SPI_MR/CSR configs
 * and run on different FreeRTOS tasks, so every bus transaction must be mutually
 * exclusive and must re-establish its own controller config when it takes the bus.
 *
 * The lock is a recursive mutex taken at the granularity of a whole FPGA operation
 * (an hal_rfid_* call) or a whole flash operation - never inside a critical section
 * (rfid.c drives the FPGA config register with interrupts disabled mid-frame, where
 * a mutex take would be illegal). spi_bus_claim() lets each user skip its (SWRST-ing)
 * re-setup when it already owned the bus, so an RFID session running alone reconfigures
 * SPI exactly once, as before. */
#ifndef PM3_SPI_BUS_H
#define PM3_SPI_BUS_H

#include <stdbool.h>

typedef enum { SPI_OWNER_NONE, SPI_OWNER_FPGA, SPI_OWNER_FLASH } spi_owner_t;

/* Create the recursive mutex. Safe to call before the scheduler starts; idempotent.
 * Call once early (hal_init) so no lazy-init race can occur between tasks. */
void spi_bus_init(void);

/* Take / release the bus (recursive - a nested take by the same task is fine).
 * Blocks until available. A no-op if the mutex somehow was never created. */
void spi_bus_lock(void);
void spi_bus_unlock(void);

/* Record `who` as the current bus owner. Returns true if the owner changed (the
 * previous transaction belonged to the other device), meaning the caller must
 * reconfigure the SPI controller (pins/MR/CSR) for its device before transferring.
 * Call only while holding the lock. */
bool spi_bus_claim(spi_owner_t who);

#endif /* PM3_SPI_BUS_H */
