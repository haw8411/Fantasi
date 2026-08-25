/* Flipper FAT backend: strong overrides of the weak vfs_fat_* hooks in
 * core/vfs.c, implemented over FatFs. core/vfs.c dispatches any VFS_BK_FAT mount
 * (the microSD at /mnt/ext0) here; this file is the only place FatFs types are
 * visible to the VFS layer.
 *
 * `leaf` is the mount-relative path the resolver produced (lfs-rooted, i.e. it
 * starts with '/'); FatFs wants a drive-qualified path "N:/leaf", so every op
 * prefixes the logical drive. */
#include "ff.h"
#include "../../core/vfs.h"
#include "../../hal/storage/fat_ramdisk.h"

#include <stdio.h>
#include <string.h>

/* A drive-qualified FatFs path is "N:" + a mount-relative leaf. The leaf comes
 * from the resolver as at most a full proto path field (64 B incl. NUL, the
 * widest the protocol carries), and the "N:" prefix needs room for a multi-digit
 * drive; size for both with slack so a valid long path never truncates. */
#define FAT_PATH_MAX 96

/* Build "N:<leaf>" (leaf already begins with '/', so we get "0:/foo" or "0:/").
 * Returns 0, or -1 if the composed path would not fit - the caller must fail the
 * operation rather than act on a silently truncated (wrong) path. */
static int fat_path(char *out, size_t n, int drv, const char *leaf)
{
    int r = snprintf(out, n, "%d:%s", drv, leaf && leaf[0] ? leaf : "/");
    return (r < 0 || (size_t)r >= n) ? -1 : 0;
}

int32_t vfs_fat_size(int drv, const char *leaf)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;
    FILINFO fno;
    fatrd_store_lock();
    FRESULT rc = f_stat(p, &fno);
    fatrd_store_unlock();
    if (rc != FR_OK || (fno.fattrib & AM_DIR)) return -1;
    return (int32_t)fno.fsize;
}

int32_t vfs_fat_pread(int drv, const char *leaf, uint32_t off, void *buf, uint32_t len)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;
    FIL f;
    fatrd_store_lock();
    if (f_open(&f, p, FA_READ) != FR_OK) { fatrd_store_unlock(); return -1; }
    if (off && f_lseek(&f, off) != FR_OK) {
        f_close(&f); fatrd_store_unlock(); return -1;
    }
    UINT br = 0;
    FRESULT r = f_read(&f, buf, len, &br);
    f_close(&f);
    fatrd_store_unlock();
    return (r == FR_OK) ? (int32_t)br : -1;
}

int vfs_fat_wchunk(int drv, const char *leaf, uint32_t off,
                   const void *buf, uint32_t len, bool last)
{
    (void)last;
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;
    FIL f;
    fatrd_store_lock();
    BYTE mode = FA_WRITE | (off == 0 ? FA_CREATE_ALWAYS : FA_OPEN_ALWAYS);
    if (f_open(&f, p, mode) != FR_OK) { fatrd_store_unlock(); return -1; }
    if (off && f_lseek(&f, off) != FR_OK) {
        f_close(&f); fatrd_store_unlock(); return -1;
    }
    UINT bw = 0;
    FRESULT wr = len ? f_write(&f, buf, len, &bw) : FR_OK;
    FRESULT cl = f_close(&f);
    fatrd_store_unlock();
    return (wr == FR_OK && bw == len && cl == FR_OK) ? 0 : -1;
}

int vfs_fat_remove(int drv, const char *leaf)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;
    fatrd_store_lock();
    FRESULT rc = f_unlink(p);
    fatrd_store_unlock();
    return rc == FR_OK ? 0 : -1;        /* also drops empty dirs */
}

int vfs_fat_mkdir(int drv, const char *leaf)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;
    fatrd_store_lock();
    FRESULT r = f_mkdir(p);
    fatrd_store_unlock();
    return (r == FR_OK || r == FR_EXIST) ? 0 : -1;
}

int vfs_fat_rename(int drv, const char *from, const char *to)
{
    char pf[FAT_PATH_MAX];
    if (fat_path(pf, sizeof pf, drv, from) != 0) return -1;
    /* f_rename ignores any drive number in the new path and reuses the old
     * volume, so the destination is passed leaf-relative. */
    fatrd_store_lock();
    FRESULT rc = f_rename(pf, (to && to[0]) ? to : "/");
    fatrd_store_unlock();
    return rc == FR_OK ? 0 : -1;
}

int vfs_fat_statfs(int drv, uint32_t *total, uint32_t *freeb)
{
    char root[8];
    snprintf(root, sizeof root, "%d:", drv);
    FATFS *fs;
    DWORD nfree;
    fatrd_store_lock();
    FRESULT rc = f_getfree(root, &nfree, &fs);
    if (rc != FR_OK) {
        fatrd_store_unlock();
        if (total) *total = 0;
        if (freeb) *freeb = 0;
        return -1;
    }
    /* KiB, not bytes: a multi-GB card's byte count overflows the uint32 statfs
     * contract. 1 sector = 512 B = half a KiB, so KiB = clusters * csize / 2. */
    uint32_t csz = fs->csize;                       /* sectors per cluster */
    uint32_t tot_clusters = fs->n_fatent - 2;
    if (total) *total = (uint32_t)(((uint64_t)tot_clusters * csz) / 2u);
    if (freeb) *freeb = (uint32_t)(((uint64_t)nfree * csz) / 2u);
    fatrd_store_unlock();
    return 0;
}

int vfs_fat_list(int drv, const char *leaf, vfs_list_cb cb, void *ctx)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;
    DIR dir;
    fatrd_store_lock();
    if (f_opendir(&dir, p) != FR_OK) { fatrd_store_unlock(); return -1; }
    FILINFO fno;
    int result = 0;
    for (;;) {
        FRESULT rc = f_readdir(&dir, &fno);
        if (rc != FR_OK) { result = -1; break; }
        if (!fno.fname[0]) break;
        bool is_dir = (fno.fattrib & AM_DIR) != 0;
        uint32_t size = is_dir ? 0 : (uint32_t)fno.fsize;
        char name[sizeof(fno.fname)];
        memcpy(name, fno.fname, sizeof(name));
        name[sizeof(name) - 1] = '\0';
        /* A response mailbox can backpressure this session. Keep the FatFs
         * serialization lock out of that path so unrelated sessions may still
         * use storage while this host is slow. */
        fatrd_store_unlock();
        cb(name, size, is_dir, ctx);
        fatrd_store_lock();
    }
    f_closedir(&dir);
    fatrd_store_unlock();
    return result;
}
