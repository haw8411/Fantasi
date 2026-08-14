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
    if (f_stat(p, &fno) != FR_OK) return -1;
    if (fno.fattrib & AM_DIR) return -1;      /* size is for regular files */
    return (int32_t)fno.fsize;
}

int32_t vfs_fat_pread(int drv, const char *leaf, uint32_t off, void *buf, uint32_t len)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;
    FIL f;
    if (f_open(&f, p, FA_READ) != FR_OK) return -1;
    if (off && f_lseek(&f, off) != FR_OK) { f_close(&f); return -1; }
    UINT br = 0;
    FRESULT r = f_read(&f, buf, len, &br);
    f_close(&f);
    return (r == FR_OK) ? (int32_t)br : -1;
}

/* Streaming write with one persistent FIL held across chunks (mirrors the lfs
 * s_file path in proto.c). off==0 truncates/creates; a later off with the file
 * already open just seeks+writes; `last` closes and flushes. */
static FIL  s_wf;
static bool s_wf_open;
static int  s_wf_drv = -1;
static char s_wf_path[FAT_PATH_MAX];

static int wclose(void)
{
    int rc = 0;
    if (s_wf_open) {
        if (f_close(&s_wf) != FR_OK) rc = -1;
        s_wf_open = false;
    }
    return rc;
}

int vfs_fat_wchunk(int drv, const char *leaf, uint32_t off,
                   const void *buf, uint32_t len, bool last)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;

    if (off == 0) {
        wclose();                                  /* drop any stale handle */
        if (f_open(&s_wf, p, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return -1;
        s_wf_open = true; s_wf_drv = drv;
        strncpy(s_wf_path, p, sizeof s_wf_path - 1);
        s_wf_path[sizeof s_wf_path - 1] = '\0';
    } else if (!s_wf_open || s_wf_drv != drv || strcmp(s_wf_path, p) != 0) {
        /* A resumed/retransmitted chunk after the handle was closed (or for a
         * different file): reopen without truncating and seek to the offset. */
        wclose();
        if (f_open(&s_wf, p, FA_WRITE | FA_OPEN_ALWAYS) != FR_OK) return -1;
        s_wf_open = true; s_wf_drv = drv;
        strncpy(s_wf_path, p, sizeof s_wf_path - 1);
        s_wf_path[sizeof s_wf_path - 1] = '\0';
    }

    if (off != f_tell(&s_wf) && f_lseek(&s_wf, off) != FR_OK) { wclose(); return -1; }

    UINT bw = 0;
    if (len && (f_write(&s_wf, buf, len, &bw) != FR_OK || bw != len)) { wclose(); return -1; }

    if (last) return wclose();
    return 0;
}

int vfs_fat_remove(int drv, const char *leaf)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;
    return (f_unlink(p) == FR_OK) ? 0 : -1;        /* also drops empty dirs */
}

int vfs_fat_mkdir(int drv, const char *leaf)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return -1;
    FRESULT r = f_mkdir(p);
    return (r == FR_OK || r == FR_EXIST) ? 0 : -1;
}

int vfs_fat_rename(int drv, const char *from, const char *to)
{
    char pf[FAT_PATH_MAX];
    if (fat_path(pf, sizeof pf, drv, from) != 0) return -1;
    /* f_rename ignores any drive number in the new path and reuses the old
     * volume, so the destination is passed leaf-relative. */
    return (f_rename(pf, (to && to[0]) ? to : "/") == FR_OK) ? 0 : -1;
}

int vfs_fat_statfs(int drv, uint32_t *total, uint32_t *freeb)
{
    char root[8];
    snprintf(root, sizeof root, "%d:", drv);
    FATFS *fs;
    DWORD nfree;
    if (f_getfree(root, &nfree, &fs) != FR_OK) {
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
    return 0;
}

void vfs_fat_list(int drv, const char *leaf, vfs_list_cb cb, void *ctx)
{
    char p[FAT_PATH_MAX];
    if (fat_path(p, sizeof p, drv, leaf) != 0) return;
    DIR dir;
    if (f_opendir(&dir, p) != FR_OK) return;
    FILINFO fno;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        bool is_dir = (fno.fattrib & AM_DIR) != 0;
        cb(fno.fname, is_dir ? 0 : (uint32_t)fno.fsize, is_dir, ctx);
    }
    f_closedir(&dir);
}
