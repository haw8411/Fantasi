/* AT32F435 internal-flash driver for the LittleFS storage region.
 *
 * The region is bank2 (0x08080000..); the firmware runs from bank1, so these
 * erase/program routines operate on a different bank than the one they execute
 * from - the AT32F435's dual-bank read-while-write lets the CPU keep fetching
 * bank1 code through a bank2 operation. We still mask interrupts across each
 * operation so no ISR (whose handler is in bank1, but could in principle chain)
 * runs mid-write. If a future layout puts code in bank2, move flash_erase/
 * flash_program into the .ramfunc section the linker already reserves.
 *
 * AT32F435 flash: 2 KB sectors, real sector-erase + word program behind an
 * unlock-key sequence, memory-mapped reads. Bank2 uses the *2 register set
 * (UNLOCK2/STS2/CTRL2/ADDR2). See platforms/proxmark5/at32f435.h. */

#include "flash_storage.h"
#include "at32f435.h"
#include <string.h>

static uint32_t s_base;

int storage_flash_init(void)
{
    /* Need the full bank1 (512 KB) + the bank2 storage region present. */
    uint16_t kb = *(volatile uint16_t *)FSIZE_BASE;
    if (kb < ((STORAGE_BASE - FLASH_BANK1_BASE) + STORAGE_SIZE) / 1024U)
        return -1;

    /* The app is linked into bank1; the region must sit above it. _eflash is
     * the end of the image's load region in flash. */
    extern uint8_t _eflash;
    if ((uint32_t)&_eflash > STORAGE_BASE)
        return -1;

    s_base = STORAGE_BASE;
    return 0;
}

uint32_t storage_flash_base(void)
{
    return s_base;
}

/* ---- bank2 flash helpers ---- */

static void flash_unlock(void)
{
    if (FLASH->CTRL2 & FLASH_CTRL_OPLK) {
        FLASH->UNLOCK2 = FLASH_UNLOCK_KEY1;
        FLASH->UNLOCK2 = FLASH_UNLOCK_KEY2;
    }
}

static void flash_lock(void)
{
    FLASH->CTRL2 |= FLASH_CTRL_OPLK;
}

static void flash_wait(void)
{
    while (FLASH->STS2 & FLASH_STS_OBF) { }
}

static int flash_check_clear(void)
{
    uint32_t sts = FLASH->STS2;
    /* Clear the sticky flags (write-1-clear). */
    FLASH->STS2 = FLASH_STS_ODF | FLASH_STS_PRGMERR | FLASH_STS_EPPERR;
    return (sts & (FLASH_STS_PRGMERR | FLASH_STS_EPPERR)) ? -1 : 0;
}

/* ---- public API ---- */

int storage_flash_read(uint32_t offset, void *buf, size_t len)
{
    if (offset + len > STORAGE_SIZE) return -1;
    memcpy(buf, (const void *)(s_base + offset), len);   /* flash is memory-mapped */
    return 0;
}

int storage_flash_erase(uint32_t page_index)
{
    if (page_index >= STORAGE_PAGE_COUNT) return -1;

    uint32_t addr = s_base + page_index * STORAGE_PAGE_SIZE;
    int rc;

    __asm volatile("cpsid i" ::: "memory");
    flash_wait();
    flash_unlock();

    FLASH->CTRL2 |= FLASH_CTRL_SECERS;
    FLASH->ADDR2  = addr;
    FLASH->CTRL2 |= FLASH_CTRL_ERSTR;
    flash_wait();
    FLASH->CTRL2 &= ~FLASH_CTRL_SECERS;

    rc = flash_check_clear();
    flash_lock();
    __asm volatile("cpsie i" ::: "memory");
    return rc;
}

int storage_flash_program(uint32_t offset, const void *buf, size_t len)
{
    if (offset + len > STORAGE_SIZE) return -1;
    if ((offset & 3) || (len & 3)) return -1;

    volatile uint32_t *dst = (volatile uint32_t *)(s_base + offset);
    const uint8_t *src = (const uint8_t *)buf;   /* may be unaligned; CM4 word reads tolerate it */
    size_t words = len / 4;
    int rc = 0;

    __asm volatile("cpsid i" ::: "memory");
    flash_wait();
    flash_unlock();

    FLASH->CTRL2 |= FLASH_CTRL_FPRGM;
    for (size_t i = 0; i < words; i++) {
        uint32_t w;
        memcpy(&w, src + i * 4, 4);
        dst[i] = w;
        flash_wait();
        if (FLASH->STS2 & (FLASH_STS_PRGMERR | FLASH_STS_EPPERR)) { rc = -1; break; }
    }
    FLASH->CTRL2 &= ~FLASH_CTRL_FPRGM;

    if (rc == 0) rc = flash_check_clear();
    else flash_check_clear();
    flash_lock();
    __asm volatile("cpsie i" ::: "memory");
    return rc;
}
