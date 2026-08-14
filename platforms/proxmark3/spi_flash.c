#include "spi_flash.h"
#include "spi_bus.h"
#include "at91sam7s512.h"

#include "FreeRTOS.h"
#include "task.h"

/* W25Q-style command set (command-port). */
#define CMD_JEDEC   0x9Fu
#define CMD_READ    0x03u   /* single-line read, 24-bit address                */
#define CMD_PROG    0x02u   /* page program, <=256 B, must not cross a page    */
#define CMD_ERASE4K 0x20u   /* 4 KB sector erase                               */
#define CMD_WREN    0x06u   /* write enable                                    */
#define CMD_RDSR    0x05u   /* read status reg 1 (bit0 = WIP/busy)             */

#define PAGE_SIZE   256u

static uint8_t  s_id[3];
static uint32_t s_size;      /* bytes; 0 = absent/not probed */

/* Bring SPI0 up for the flash on NPCS2/PA10 - identical sequence to the port's
 * unique-id reader (hal.c) and the flashmem driver: PA11-14 -> peripheral A,
 * PA10 -> peripheral B (NPCS2), master/fixed/MODFDIS, CSR2 = ~24 MHz, mode 0,
 * CSAAT (CS held between transfers), 8-bit. Called under the bus lock whenever the
 * flash (re)takes the bus, so a prior FPGA config is fully overwritten. */
static void flash_spi_setup(void)
{
    AT91C_BASE_PMC->PMC_PCER = (1u << AT91C_ID_SPI);
    AT91C_BASE_PIOA->PIO_PDR = (1u<<10)|(1u<<11)|(1u<<12)|(1u<<13)|(1u<<14);
    AT91C_BASE_PIOA->PIO_ASR = (1u<<11)|(1u<<12)|(1u<<13)|(1u<<14);
    AT91C_BASE_PIOA->PIO_BSR = (1u<<10);

    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SWRST;
    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SWRST;
    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SPIEN;

    AT91C_BASE_SPI->SPI_MR = ((~(1u << 2) & 0xF) << 16) |
                              (1u << 4) | AT91C_SPI_PS_FIXED | AT91C_SPI_MSTR;
    AT91C_BASE_SPI->SPI_CSR[2] = (2u << 8) | AT91C_SPI_BITS_8 |
                                  (1u << 1) |  /* NCPHA -> SPI mode 0 */
                                  (1u << 3);   /* CSAAT -> hold CS across the command */
}

/* One byte out / in. Pass AT91C_SPI_LASTXFER on the final byte of a command to
 * deassert CS afterwards (CSAAT keeps it low otherwise). The RDRF wait is bounded:
 * a healthy transfer completes in <1 us, so 100k spins (~ms) is a huge margin, and a
 * mis-configured/contended bus returns junk instead of hanging the whole system. */
static inline uint8_t xf(uint32_t data)
{
    AT91C_BASE_SPI->SPI_TDR = data;
    uint32_t g = 0;
    while (!(AT91C_BASE_SPI->SPI_SR & AT91C_SPI_RDRF) && ++g < 100000u) { }
    return (uint8_t)(AT91C_BASE_SPI->SPI_RDR & 0xFF);
}

static void write_enable(void)
{
    xf(CMD_WREN | AT91C_SPI_LASTXFER);
}

/* Poll status until WIP (busy) clears, yielding between polls so a ~45 ms erase never
 * starves other tasks (the bus lock is held throughout - intended mutual exclusion).
 * Bounded (~2 s >> any erase) so a dead chip can't hang forever. */
static void wait_busy(void)
{
    xf(CMD_RDSR);
    for (uint32_t g = 0; g < 2000u; g++) {
        if (!(xf(0xFF) & 0x01u)) break;
        vTaskDelay(1);
    }
    xf(0xFF | AT91C_SPI_LASTXFER);              /* release CS */
}

/* --- must hold the bus lock + have claimed SPI_OWNER_FLASH before these --- */

static void send_addr(uint8_t cmd, uint32_t addr)
{
    xf(cmd);
    xf((addr >> 16) & 0xFF);
    xf((addr >>  8) & 0xFF);
    xf( addr        & 0xFF);   /* more bytes follow, so no LASTXFER here */
}

static void prog_page(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    write_enable();
    send_addr(CMD_PROG, addr);
    for (uint32_t i = 0; i < len; i++)
        xf(i == len - 1 ? ((uint32_t)buf[i] | AT91C_SPI_LASTXFER) : buf[i]);
    wait_busy();
}

bool spi_flash_init(void)
{
    spi_bus_lock();
    if (spi_bus_claim(SPI_OWNER_FLASH)) flash_spi_setup();

    xf(CMD_JEDEC);
    uint8_t m = xf(0xFF);
    uint8_t d = xf(0xFF);
    uint8_t c = xf(0xFF | AT91C_SPI_LASTXFER);

    spi_bus_unlock();

    uint16_t jedec = ((uint16_t)d << 8) | c;
    if (jedec == 0 || jedec == 0xFFFF) { s_size = 0; return false; }

    s_id[0] = m; s_id[1] = d; s_id[2] = c;
    s_size  = (1u << (jedec & 0x0Fu)) * 64u * 1024u;
    return true;
}

uint32_t spi_flash_size(void) { return s_size; }

void spi_flash_unique_id(uint8_t out[8])
{
    spi_bus_lock();
    if (spi_bus_claim(SPI_OWNER_FLASH)) flash_spi_setup();

    xf(0x4B);                                    /* Read Unique ID: 1 cmd + 4 dummy + 8 */
    xf(0xFF); xf(0xFF); xf(0xFF); xf(0xFF);
    for (int i = 0; i < 8; i++)
        out[i] = xf(i == 7 ? (0xFF | AT91C_SPI_LASTXFER) : 0xFF);

    spi_bus_unlock();
}

void spi_flash_id(uint8_t out[3])
{
    out[0] = s_id[0]; out[1] = s_id[1]; out[2] = s_id[2];
}

int spi_flash_read(uint32_t addr, void *buf, uint32_t len)
{
    if (len == 0) return 0;
    if (s_size && addr + len > s_size) return -1;

    spi_bus_lock();
    if (spi_bus_claim(SPI_OWNER_FLASH)) flash_spi_setup();

    send_addr(CMD_READ, addr);
    uint8_t *b = buf;
    for (uint32_t i = 0; i < len; i++)
        b[i] = xf(i == len - 1 ? (0xFF | AT91C_SPI_LASTXFER) : 0xFF);

    spi_bus_unlock();
    return 0;
}

int spi_flash_program(uint32_t addr, const void *buf, uint32_t len)
{
    if (len == 0) return 0;
    if (s_size && addr + len > s_size) return -1;

    spi_bus_lock();
    if (spi_bus_claim(SPI_OWNER_FLASH)) flash_spi_setup();

    const uint8_t *b = buf;
    while (len) {
        uint32_t pg = PAGE_SIZE - (addr & (PAGE_SIZE - 1));   /* to next page boundary */
        if (pg > len) pg = len;
        prog_page(addr, b, pg);
        addr += pg; b += pg; len -= pg;
    }

    spi_bus_unlock();
    return 0;
}

int spi_flash_erase4k(uint32_t addr)
{
    if (s_size && addr >= s_size) return -1;

    spi_bus_lock();
    if (spi_bus_claim(SPI_OWNER_FLASH)) flash_spi_setup();

    write_enable();
    xf(CMD_ERASE4K);
    xf((addr >> 16) & 0xFF);
    xf((addr >>  8) & 0xFF);
    xf((addr        & 0xFF) | AT91C_SPI_LASTXFER);
    wait_busy();

    spi_bus_unlock();
    return 0;
}
