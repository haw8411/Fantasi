/* AT91SAM7S512 flash driver for LittleFS storage.
 *
 * Storage occupies flash plane 1 (0x140000-0x17FFFF, 256 KB).
 * Only available on the S512 variant - S256 has one plane and no
 * room for storage.
 *
 * AT91SAM7S flash pages are 256 bytes. The WP (Write Page) command
 * auto-erases the page before programming, so erase and program are
 * both implemented as full-page writes via EFC1. */

#include "flash_storage.h"
#include "at91sam7s512.h"
#include <string.h>

#define PLANE1_BASE      0x00140000U
#define EFC1_KEY         (0x5AU << 24)

static uint32_t s_base;

int storage_flash_init(void)
{
    uint32_t cidr = *AT91C_DBGU_CIDR;
    uint32_t nvpsiz = (cidr >> 8) & 0xFU;
    if (nvpsiz != 10)
        return -1;

    s_base = PLANE1_BASE;
    return 0;
}

uint32_t storage_flash_base(void)
{
    return s_base;
}

int storage_flash_read(uint32_t offset, void *buf, size_t len)
{
    if (offset + len > STORAGE_SIZE) return -1;
    memcpy(buf, (const void *)(s_base + offset), len);
    return 0;
}

static void efc1_write_page(uint32_t page_addr)
{
    uint32_t page_num = (page_addr - PLANE1_BASE) / STORAGE_PAGE_SIZE;
    *AT91C_EFC1_FCR = EFC1_KEY
                    | (page_num << 8)
                    | AT91C_MC_FCMD_START_PROG;
    while (!(*AT91C_EFC1_FSR & AT91C_MC_FRDY)) {}
}

int storage_flash_erase(uint32_t page_index)
{
    if (page_index >= STORAGE_PAGE_COUNT) return -1;

    uint32_t addr = s_base + page_index * STORAGE_PAGE_SIZE;
    volatile uint32_t *dst = (volatile uint32_t *)addr;
    for (uint32_t i = 0; i < STORAGE_PAGE_SIZE / 4; i++)
        dst[i] = 0xFFFFFFFFU;

    efc1_write_page(addr);
    return 0;
}

int storage_flash_program(uint32_t offset, const void *buf, size_t len)
{
    if (offset + len > STORAGE_SIZE) return -1;
    if ((offset & 3) || (len & 3)) return -1;

    const uint8_t *src = (const uint8_t *)buf;

    while (len > 0) {
        uint32_t page_start = offset & ~(uint32_t)(STORAGE_PAGE_SIZE - 1);
        uint32_t page_off = offset - page_start;
        uint32_t chunk = STORAGE_PAGE_SIZE - page_off;
        if (chunk > len) chunk = len;

        uint8_t page_buf[STORAGE_PAGE_SIZE];
        memcpy(page_buf, (const void *)(s_base + page_start), STORAGE_PAGE_SIZE);
        memcpy(&page_buf[page_off], src, chunk);

        volatile uint32_t *dst = (volatile uint32_t *)(s_base + page_start);
        const uint32_t *words = (const uint32_t *)page_buf;
        for (uint32_t i = 0; i < STORAGE_PAGE_SIZE / 4; i++)
            dst[i] = words[i];

        efc1_write_page(s_base + page_start);

        offset += chunk;
        src += chunk;
        len -= chunk;
    }
    return 0;
}
