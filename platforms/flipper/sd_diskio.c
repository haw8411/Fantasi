/* FatFs <-> Flipper SD glue. Bridges the FatFs diskio callbacks to sd_spi.c.
 * Only one physical drive (pdrv 0) exists: the microSD card. */
#include "ff.h"
#include "diskio.h"
#include "sd_spi.h"

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    return sd_spi_ready() ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    sd_spi_init();
    return sd_spi_ready() ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0) return RES_PARERR;
    if (!sd_spi_ready()) return RES_NOTRDY;
    return sd_spi_read((uint32_t)sector, buff, count) == 0 ? RES_OK : RES_ERROR;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0) return RES_PARERR;
    if (!sd_spi_ready()) return RES_NOTRDY;
    return sd_spi_write((uint32_t)sector, buff, count) == 0 ? RES_OK : RES_ERROR;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0) return RES_PARERR;
    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;                 /* sd_spi_write already waits out the program */
    case GET_SECTOR_COUNT:
        *(LBA_t *)buff = sd_spi_sector_count();
        return RES_OK;
    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        return RES_OK;
    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1;            /* erase-block size unknown -> 1 (in sectors) */
        return RES_OK;
    default:
        return RES_PARERR;
    }
}
