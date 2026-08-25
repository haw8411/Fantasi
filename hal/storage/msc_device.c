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
/* Switch-mode platforms may use this to keep a personality-change grace period
 * open until queued block I/O has actually gone quiet. Composite targets need no
 * bookkeeping, so the default remains free. */
__attribute__((weak)) void hal_on_msc_activity(void) {}

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
    /* Signal a media change (SCSI UNIT ATTENTION, "medium may have changed") when
     * the filesystem was mutated by a non-MSC path since the host last polled. The
     * host clears the attention by re-reading, dropping its stale cached view - so a
     * file written over proto/BLE becomes visible to a host that has the drive
     * mounted, without any unmount/remount. Reported once per change, then ready. */
    if (fatrd_media_changed()) {
        tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
        return false;
    }
    return true;                              /* synthetic FAT is always ready */
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_count = fatrd_sector_count();
    *block_size  = FATRD_SECTOR_SIZE;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void *buffer, uint32_t bufsize)
{
    hal_on_msc_activity();
    if (fatrd_read(lba, offset, buffer, bufsize) < 0) {
        tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);   /* UNRECOVERED READ ERROR */
        return -1;
    }
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t *buffer, uint32_t bufsize)
{
    hal_on_msc_activity();
    /* A failed capture (low heap or full volume) must report a definitive SCSI error,
     * else the host retries the same block indefinitely. MEDIUM ERROR / WRITE ERROR makes
     * it abort the copy cleanly ("I/O error" / disk full). */
    if (fatrd_write(lba, offset, buffer, bufsize) < 0) {
        tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x0C, 0x00);   /* WRITE ERROR */
        return -1;
    }
    return (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                        void *buffer, uint16_t bufsize)
{
    (void)buffer; (void)bufsize;
    /* SYNCHRONIZE CACHE (10/16): the host sends this on flush/unmount. Use it to
     * commit any complete host-over-MSC file writes durably to LittleFS, then ack. */
    if (scsi_cmd[0] == 0x35 || scsi_cmd[0] == 0x91) {
        hal_on_msc_activity();
        if (!fatrd_sync()) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x0C, 0x00);
            return -1;
        }
        return 0;
    }
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition,
                           bool start, bool load_eject)
{
    (void)lun; (void)power_condition;
    /* The host flushes with SYNCHRONIZE CACHE before eject. Eject must still fire
     * hal_on_msc_eject() so switch-mode platforms (PM3) leave MSC and re-enumerate
     * as CDC, then discard this mount's bounded staging/model state. */
    if (load_eject && !start) { hal_on_msc_eject(); fatrd_release(); }   /* host ejected: drop the ~6 KB FAT model */
    return true;
}

/* USB unplugged: the host can no longer mount the drive, so free the synthetic-FAT model. Runs on the usb
 * task (tud_task), same context as the MSC read/write callbacks, so it can't race a transfer.
 * usb_power_umount (hal/tinyusb/usb_power.c) releases the USB sleep-inhibitor vote; weak no-op
 * for platforms that don't build the vote glue. */
__attribute__((weak)) void usb_power_umount(void) {}
void tud_umount_cb(void) { usb_power_umount(); fatrd_release(); }

void tud_msc_write10_complete_cb(uint8_t lun)
{
    (void)lun;
}

#endif
