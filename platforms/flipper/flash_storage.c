/* STM32WB55 internal flash driver for the LittleFS storage region.
 *
 * The storage region is placed at the top of available flash, just
 * below the BLE secure region (SFSA). This keeps it far from the
 * firmware, so DFU updates never touch the storage pages.
 *
 * Flash is memory-mapped so reads are trivial. Programming is 8-byte
 * double-word aligned; erasing is per 4 KB page. */

#include "flash_storage.h"
#include "ble.h"
#include "stm32wbxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define FLASH_KEY1  0x45670123U
#define FLASH_KEY2  0xCDEF89ABU

#define FLASH_ALL_ERRORS \
    (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | \
     FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | \
     FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR | FLASH_SR_OPTVERR)

/* Only check for real programming errors - OPERR, MISERR, FASTERR,
 * OPTVERR, RDERR are spuriously set by CPU2 or errata 2.2.9. */
#define FLASH_PROG_ERRORS \
    (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR)

static uint32_t s_base;

static void flash_unlock(void);

/* ---- CPU1/CPU2 flash↔radio coordination (mirrors furi_hal_flash.c) ----
 * The STM32WB shares a single flash bank between both cores; a CPU1 erase or
 * program stalls the bus for BOTH cores. The reference firmware follows a
 * precise ST sequence (AN5289 / STM32CubeWB flash_driver.c) so the BLE radio
 * (CPU2) stays alive AND the host doesn't lose in-flight writes during a long
 * erase. The critical mechanism: SHCI_C2_FLASH_EraseActivity(ON) makes CPU2
 * RESCHEDULE its connection events away from the erase window - so the host
 * gets no events to send into during the stall and simply waits, instead of
 * its writes being silently dropped. The full handshake is required: without
 * the flash mutex (SEM2) and the PESD wait, EraseActivity does not take effect
 * and CPU2 is not actually kept off the bus. SEM7 alone is not sufficient.
 *
 * Semaphore roles (hsem_map.h):
 *   SEM2  CFG_HW_FLASH_SEMID                  - flash controller ownership
 *   SEM6  CFG_HW_BLOCK_FLASH_REQ_BY_CPU1_SEMID - CPU1 blocks CPU2 (we check it)
 *   SEM7  CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID - CPU2 blocks CPU1; CPU1 takes it
 *                                                to keep CPU2 out of flash. */
#define HSEM_FLASH_SEM      2
#define HSEM_BLOCK_BY_CPU1  6
#define HSEM_BLOCK_BY_CPU2  7
#define HSEM_COREID_CPU1    0x04U

static inline int hsem_lock(int sem)   /* 0 = locked OK (cf. LL_HSEM_1StepLock) */
{
    return (HSEM->RLR[sem] !=
            (HSEM_RLR_LOCK_Msk | (HSEM_COREID_CPU1 << HSEM_RLR_COREID_Pos))) ? 1 : 0;
}
static inline void hsem_unlock(int sem)
{
    HSEM->R[sem] = (HSEM_COREID_CPU1 << HSEM_R_COREID_Pos);
}
static inline int hsem_is_locked(int sem)
{
    return (HSEM->R[sem] & HSEM_R_LOCK_Msk) ? 1 : 0;
}

static void flush_icache(void)
{
    if (FLASH->ACR & FLASH_ACR_ICEN) {
        FLASH->ACR &= ~FLASH_ACR_ICEN;
        FLASH->ACR |= FLASH_ACR_ICRST;
        FLASH->ACR &= ~FLASH_ACR_ICRST;
        FLASH->ACR |= FLASH_ACR_ICEN;
    }
}

/* Acquire flash for a CPU1 op, coordinating with CPU2 when it is running.
 * Returns true if CPU2 coordination was used - then the caller is INSIDE a
 * FreeRTOS critical section and must call flash_end(erase, true). When CPU2 is
 * down (boot-time format) there is no radio to protect: just unlock. */
static bool flash_begin(bool erase)
{
    /* Coordinate with CPU2 whenever it is RUNNING - not just when BLE is active.
     * `ble off` leaves CPU2 running (it only stops advertising), so a write on
     * the uncoordinated path would still race CPU2 and corrupt a double-word. */
    if (!ble_cpu2_running()) {
        flash_unlock();
        return false;
    }
    while (hsem_lock(HSEM_FLASH_SEM) != 0)   /* SEM2: flash controller mutex */
        taskYIELD();
    flash_unlock();
    if (erase)
        ble_flash_erase_activity(1);          /* CPU2 reschedules radio off the erase */
    for (volatile int d = 0; d < 120; d++) __NOP();  /* ~5 µs core2 flag protect */
    for (;;) {
        while (FLASH->SR & FLASH_SR_PESD)     /* CPU2 currently in flash - wait */
            taskYIELD();
        taskENTER_CRITICAL();
        if (hsem_is_locked(HSEM_BLOCK_BY_CPU1)) {
            taskEXIT_CRITICAL(); taskYIELD(); continue;
        }
        if (hsem_lock(HSEM_BLOCK_BY_CPU2) != 0) {   /* CPU2 holds it - back off */
            taskEXIT_CRITICAL(); taskYIELD(); continue;
        }
        break;   /* holding SEM2 + SEM7, inside critical section */
    }
    return true;
}

static void flash_end(bool erase, bool c2)
{
    if (c2) {
        hsem_unlock(HSEM_BLOCK_BY_CPU2);
        taskEXIT_CRITICAL();
        while (FLASH->SR & FLASH_SR_BSY) {}
        if (erase)
            ble_flash_erase_activity(0);
        FLASH->CR |= FLASH_CR_LOCK;
        hsem_unlock(HSEM_FLASH_SEM);
    } else {
        FLASH->CR |= FLASH_CR_LOCK;
    }
}

int storage_flash_init(void)
{
    uint32_t sfsa = (FLASH->SFR & FLASH_SFR_SFSA_Msk) >> FLASH_SFR_SFSA_Pos;
    uint32_t secure_start = FLASH_BASE + sfsa * STORAGE_PAGE_SIZE;

    if (secure_start < FLASH_BASE + STORAGE_SIZE)
        return -1;

    s_base = secure_start - STORAGE_SIZE;
    s_base &= ~(STORAGE_PAGE_SIZE - 1);

    extern uint8_t _eflash;
    if ((uint32_t)&_eflash > s_base)
        return -1;

    if (FLASH->SR & FLASH_ALL_ERRORS)
        FLASH->SR = FLASH_ALL_ERRORS;

    return 0;
}

uint32_t storage_flash_base(void) { return s_base; }

static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        __ISB();
        FLASH->KEYR = FLASH_KEY2;
    }
}

int storage_flash_read(uint32_t offset, void *buf, size_t len)
{
    if (offset + len > STORAGE_SIZE) return -1;
    memcpy(buf, (const void *)(s_base + offset), len);
    return 0;
}

int storage_flash_erase(uint32_t page_index)
{
    if (page_index >= STORAGE_PAGE_COUNT) return -1;

    uint32_t abs_page = (s_base - FLASH_BASE) / STORAGE_PAGE_SIZE
                      + page_index;

    bool c2 = flash_begin(true);   /* EraseActivity ON + SEM2/SEM7 (in critical) */
    FLASH->SR = FLASH_ALL_ERRORS;
    while (FLASH->SR & FLASH_SR_BSY) {}
    MODIFY_REG(FLASH->CR, FLASH_CR_PNB,
               (abs_page << FLASH_CR_PNB_Pos) | FLASH_CR_PER | FLASH_CR_STRT);
    while (FLASH->SR & FLASH_SR_BSY) {}
    int err = (FLASH->SR & FLASH_PROG_ERRORS) ? -1 : 0;
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER | FLASH_CR_PNB);
    flush_icache();                /* data consistency after erase */
    flash_end(true, c2);
    return err;
}

int storage_flash_program(uint32_t offset, const void *buf, size_t len)
{
    if (offset + len > STORAGE_SIZE) return -1;
    if ((offset & 7) || (len & 7)) return -1;

    volatile uint32_t *dst = (volatile uint32_t *)(s_base + offset);
    const uint32_t *src = (const uint32_t *)buf;
    size_t dwords = len / 8;
    int err = 0;

    /* ONE flash<->radio handshake (SEM2 + PESD wait + SEM7 + critical section)
     * per program call, then program every double-word inside it.
     * Avoid these implementations:
     *   - per-double-word begin/end (mirroring furi_hal_flash_write_dword):
     *     a 16 KB BLE upload writes ~2048 double-words, i.e. ~2048 full radio
     *     handshakes while the link is live. That contention stalls the
     *     upload-ack path and makes BLE transfers unreliable; one handshake per
     *     call avoids it.
     *   - holding one lock but programming raw 0xFF padding: see skip below.
     * A single short lock per lfs prog call keeps up with the BLE upload AND
     * leaves the radio enough windows (lfs writes are small; the 4 KB MSC
     * page write-back is the only large caller and it tolerates one lock since
     * it is not concurrent with a high-rate BLE stream). */
    bool c2 = flash_begin(false);
    FLASH->SR = FLASH_ALL_ERRORS;
    SET_BIT(FLASH->CR, FLASH_CR_PG);

    for (size_t i = 0; i < dwords; i++) {
        /* NEVER program an all-ones double-word. The data would not change
         * (erased flash already reads 0xFF), but the STM32WB flash ECC for
         * 0xFFFFFFFFFFFFFFFF is NOT all-ones, so a program cycle writes ECC
         * bits and leaves the location NON-erased: a later attempt to program
         * real data there fails with PROGERR ("not previously erased"), even
         * though the data still reads 0xFF. lfs never asks to program padding,
         * but the MSC page write-back programs a full 4 KB page including its
         * 0xFF padding; without this skip, every padded double-word becomes
         * silently un-writable and lfs eventually fails to append metadata into
         * what it correctly believes is free, erased space. Skipping keeps
         * padding truly erased and readable as 0xFF. */
        if (src[0] == 0xFFFFFFFFu && src[1] == 0xFFFFFFFFu) {
            dst += 2;
            src += 2;
            continue;
        }

        /* The two 32-bit stores of a double-word MUST be consecutive flash
         * accesses; the critical section only raises BASEPRI and does NOT mask
         * the high-priority USB IRQ on this port, so mask ALL interrupts
         * (PRIMASK) across the two stores. */
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        dst[0] = src[0];
        __ISB();
        dst[1] = src[1];
        while (FLASH->SR & FLASH_SR_BSY) {}
        __set_PRIMASK(pm);

        if (FLASH->SR & FLASH_PROG_ERRORS) {
            err = -1;
            break;
        }
        dst += 2;
        src += 2;
    }

    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
    flash_end(false, c2);
    return err;
}
