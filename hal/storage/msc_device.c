/* TinyUSB MSC callbacks - exposes device storage as a USB mass-storage block
 * device. The single LUN is a synthetic FAT16 view of the whole VFS (root mirrors
 * the LittleFS root plus the RAMFS/ and APPS/ folders); fatrd_* synthesizes reads
 * and captures writes. All targets build this (every platform defines
 * FANTASI_FAT_STORAGE). */

#include "tusb.h"

#if CFG_TUD_MSC

#include "hal_storage.h"
#include "../../hal/hal.h"
#include "fat_ramdisk.h"
#include <string.h>

__attribute__((weak)) void hal_on_msc_eject(void) {}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                        uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;
    memcpy(vendor_id,   "Fantasi ", 8);
    memcpy(product_id,  "Storage (FAT)   ", 16);
    memcpy(product_rev, "1.0 ", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    return true;                              /* synthetic FAT is always ready */
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_count = FATRD_SECTOR_COUNT;
    *block_size  = FATRD_SECTOR_SIZE;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void *buffer, uint32_t bufsize)
{
    (void)lun;
    return fatrd_read(lba, offset, buffer, bufsize) < 0 ? -1 : (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t *buffer, uint32_t bufsize)
{
    (void)lun;
    return fatrd_write(lba, offset, buffer, bufsize) < 0 ? -1 : (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                        void *buffer, uint16_t bufsize)
{
    (void)buffer; (void)bufsize;
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition,
                           bool start, bool load_eject)
{
    (void)lun; (void)power_condition;
    /* FAT writes are committed synchronously, so no flush is needed - but eject
     * must still fire hal_on_msc_eject() so switch-mode platforms (PM3) leave MSC
     * and re-enumerate as CDC. (No-op on composite FZ/CU.) */
    if (load_eject && !start) { hal_on_msc_eject(); fatrd_release(); }   /* host ejected: drop the ~6 KB FAT model */
    return true;
}

/* USB unplugged: the host can no longer mount the drive, so free the synthetic-FAT model. Runs on the usb
 * task (tud_task), same context as the MSC read/write callbacks, so it can't race a transfer. */
void tud_umount_cb(void) { fatrd_release(); }

void tud_msc_write10_complete_cb(uint8_t lun)
{
    (void)lun;
}

#endif
