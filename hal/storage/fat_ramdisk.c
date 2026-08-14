#include "fat_ramdisk.h"
#include "../../core/vfs.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>

/* Synthetic FAT32 (with VFAT long filenames) view of the whole VFS. The root
 * mirrors the LittleFS root (files + one level of subdirectories, e.g. /apps);
 * targets with a RAM filesystem also get a synthetic /ramfs folder. Reads
 * synthesize boot/FAT/directories/file-data from the VFS; writes parse directory
 * entries (create/delete) and stage data sectors, flushing a complete file
 * through the VFS (which routes /ramfs to RAM and everything else to flash).
 * Long/mixed-case names round-trip via VFAT LFN entries; each item also carries
 * a generated unique 8.3 short name the LFN entries hang off. */

/* ---- FAT32 geometry (runtime) ----
 * Unlike FAT16 there is no fixed root-directory region: the root is an ordinary
 * cluster chain (start = g_root_clus, normally 2). Layout:
 *   lba 0 boot | 1 FSInfo | ... | g_fat_lba.. FAT | g_data_lba.. data (cluster 2)
 * The dimensions are computed per model build (geom_compute) rather than fixed at
 * compile time, so the same generator serves a small internal-only volume and,
 * later, a card-sized volume with a passthrough window. */
#define BPS         512u
#define NFAT        1u
#define RSVD        32u                      /* boot + FSInfo + backup boot + slack */
#define EPS         16u                      /* dir entries per 512-byte sector */
#define FSINFO_LBA  1u
#define ROOT_CLUS   2u                       /* FAT32 root directory start cluster */

/* Every synthetic directory is sized to its live entries plus this many spare
 * clusters (16 free 32-byte slots each). Without slack, a directory whose clusters
 * are full forces the host to extend it into a freshly-allocated cluster the model
 * never reserved for that directory; the host then writes the new file's entry
 * there and the model, not recognizing it as directory space, captures it as file
 * data and loses it. One spare cluster lets the host add entries in-place, into
 * clusters the model routes to write_dir. The next model rebuild restores the slack. */
#define DIR_SLACK_CLUS 1u

/* FAT32 requires at least 65525 data clusters. An internal-only drive is tiny
 * (a few hundred KB), so we simply declare a comfortably-valid fixed count; the
 * synthetic model uses only the low handful and the rest reads as free. */
#define MIN_FAT32_CLUSTERS 65525u
#define INTERNAL_CLUSTERS  66000u

static uint32_t g_sec_per_clus = 1;          /* 1 for internal-only; card's when passthrough */
static uint32_t g_clusters     = INTERNAL_CLUSTERS;   /* total data clusters */
static uint32_t g_fatsz;                     /* sectors per FAT */
static uint32_t g_fat_lba      = RSVD;
static uint32_t g_data_lba;                  /* first data sector (== cluster 2) */
static uint32_t g_total_sec;                 /* whole-volume sector count */

/* Passthrough window (0 when internal-only). Clusters [2 .. g_pt_max] map verbatim
 * to the card's data region; merged FAT sectors [0 .. g_pt_fatsec) mirror the
 * card's FAT sectors. Internal synthetic content lives at clusters >= g_int_first
 * (128-aligned so no FAT sector straddles the boundary). */
static uint32_t g_pt_max;                     /* highest passthrough cluster (0 = none) */
static uint32_t g_pt_fatsec;                  /* count of card-mirrored FAT sectors */
static uint32_t g_pt_data_lba;                /* card LBA of cluster 2 */
static uint32_t g_pt_fat_lba;                 /* card LBA of the FAT */
static uint32_t g_int_first    = 2;           /* first internal (synthetic) cluster */
static uint32_t g_ext_root_clus;              /* card root cluster (for the ext0 entry) */
static uint32_t g_card_free;                  /* free clusters on the card (for FSInfo) */

/* ---- Weak external-passthrough hooks (see fat_ramdisk.h). A platform with a
 * real-FAT device overrides these; the defaults keep the drive internal-only. --- */
__attribute__((weak)) bool fatrd_ext_present(fatrd_ext_t *out) { (void)out; return false; }
__attribute__((weak)) int  fatrd_ext_read(uint32_t lba, uint8_t *buf)
{ (void)lba; (void)buf; return -1; }
__attribute__((weak)) int  fatrd_ext_read_multi(uint32_t lba, uint8_t *buf, uint32_t count)
{   /* default: loop the single-sector hook (a platform can override with a real
     * multi-block read). */
    for (uint32_t i = 0; i < count; i++)
        if (fatrd_ext_read(lba + i, buf + i * BPS) != 0) return -1;
    return 0;
}
__attribute__((weak)) int  fatrd_ext_write(uint32_t lba, const uint8_t *buf)
{ (void)lba; (void)buf; return -1; }

static void put32(uint8_t *p, uint32_t v);   /* LE writer, defined below */

/* Write `len` bytes at `offset` into card sector `lba` (read-modify-write for a
 * partial sector; MSC writes are normally whole 512-byte sectors). */
static int ext_write_sector(uint32_t lba, uint32_t offset, const uint8_t *buf, uint32_t len)
{
    uint8_t sec[BPS];
    if (offset == 0 && len == BPS) {
        return fatrd_ext_write(lba, buf);
    }
    if (fatrd_ext_read(lba, sec) != 0) memset(sec, 0, BPS);
    memcpy(sec + offset, buf, len);
    return fatrd_ext_write(lba, sec);
}

/* Next cluster in the card's FAT chain (0x0FFFFFFF at end / on error). */
static uint32_t card_fat_next(uint32_t clus)
{
    uint8_t s[BPS];
    if (fatrd_ext_read(g_pt_fat_lba + clus / 128u, s) != 0) return 0x0FFFFFFF;
    uint32_t i = (clus % 128u) * 4u;
    return (s[i] | (s[i+1] << 8) | (s[i+2] << 16) | ((uint32_t)s[i+3] << 24)) & 0x0FFFFFFF;
}

/* Read a file's `size` bytes from the card, following its FAT chain from `start`
 * (used to reconcile an internal-subtree file whose data the host wrote into the
 * card scratch). Returns 0 on success. */
static int ext_read_file(uint32_t start, uint32_t size, uint8_t *dst)
{
    uint32_t got = 0, clus = start;
    uint8_t s[BPS];
    while (got < size && clus >= 2 && clus <= g_pt_max) {
        for (uint32_t intra = 0; intra < g_sec_per_clus && got < size; intra++) {
            if (fatrd_ext_read(g_pt_data_lba + (clus - 2u) * g_sec_per_clus + intra, s) != 0)
                return -1;
            uint32_t n = size - got; if (n > BPS) n = BPS;
            memcpy(dst + got, s, n); got += n;
        }
        uint32_t nx = card_fat_next(clus);
        if (nx < 2 || nx >= 0x0FFFFFF8) break;
        clus = nx;
    }
    return got == size ? 0 : -1;
}

/* Free the card cluster chain starting at `start` (mark each entry 0 in the card
 * FAT). Used after reconciling an internal-subtree file whose data transited the
 * card scratch, so those clusters don't leak as lost clusters on the card. */
static void ext_free_chain(uint32_t start)
{
    uint32_t clus = start;
    while (clus >= 2 && clus <= g_pt_max) {
        uint32_t nx = card_fat_next(clus);
        uint8_t s[BPS];
        if (fatrd_ext_read(g_pt_fat_lba + clus / 128u, s) == 0) {
            put32(s + (clus % 128u) * 4u, 0);
            fatrd_ext_write(g_pt_fat_lba + clus / 128u, s);
        }
        if (nx < 2 || nx >= 0x0FFFFFF8) break;
        clus = nx;
    }
}

static void geom_compute(void)
{
    g_fat_lba   = RSVD;
    g_fatsz     = (((g_clusters + 2u) * 4u) + BPS - 1u) / BPS;   /* 32-bit entries */
    g_data_lba  = RSVD + NFAT * g_fatsz;
    g_total_sec = g_data_lba + g_clusters * g_sec_per_clus;
}

/* Choose the volume geometry. Today this is always the fixed internal-only FAT32
 * volume; when a real-FAT external device is mounted, this becomes card-sized with
 * a low-cluster passthrough window (added with step 2). Kept in one place so the
 * MSC capacity callback and the model always agree. */
static void select_geometry(void)
{
    fatrd_ext_t e;
    if (fatrd_ext_present(&e) && e.sec_per_clus && e.data_clusters) {
        /* Card-sized FAT32 overlay. The card keeps its own cluster numbers in the
         * low range [2 .. g_pt_max] (no rebasing); the internal synthetic content
         * starts at the first cluster past the card's FAT, rounded to a merged-FAT
         * sector boundary so no FAT sector mixes card and internal entries. */
        g_sec_per_clus = e.sec_per_clus;
        g_pt_max       = e.data_clusters + 1;        /* card clusters: 2 .. g_pt_max */
        g_pt_data_lba  = e.data_lba;
        g_pt_fat_lba   = e.fat_lba;
        g_ext_root_clus = e.root_clus;
        g_card_free    = e.free_clusters;
        g_int_first    = g_pt_max + 1;               /* internal starts right after the card (no gap) */
        g_pt_fatsec    = 0;                           /* unused; the FAT dispatch checks g_pt_max per sector */
        g_clusters     = (g_int_first - 2u) + INTERNAL_CLUSTERS;
    } else {
        g_sec_per_clus = 1;
        g_pt_max       = 0;
        g_pt_fatsec    = 0;
        g_int_first    = 2;
        g_ext_root_clus = 0;
        g_card_free    = 0;
        g_clusters     = INTERNAL_CLUSTERS;
    }
    geom_compute();
}

static bool cluster_is_passthrough(uint32_t c) { return g_pt_max && c >= 2 && c <= g_pt_max; }

/* Runtime volume size for the MSC capacity callback (block_count). */
uint32_t fatrd_sector_count(void)
{
    select_geometry();
    return g_total_sec;
}

/* The model holds the whole directory tree in RAM (to lay out FAT clusters), so it
 * grows to fit the filesystem rather than capping at a fixed count - the only real
 * limit is heap (OOM -> blank drive). INIT_* are just the starting sizes;
 * build_model doubles the capacity and refills whenever a pass overflows. */
#define INIT_DIRS  8u
#define INIT_FILES 32u
#define VPATH_MAX  96
#define LFN_CHARS  13                        /* name chars per LFN entry */

typedef struct { char vpath[VPATH_MAX]; uint32_t start, nclus; int parent; bool is_ext; } syndir_t;
typedef struct { char vpath[VPATH_MAX]; uint32_t size, start, nclus; int parent; } synfile_t;

static syndir_t  *s_dir;     /* the synthetic-FAT model - heap, allocated on first MSC access, freed on */
static synfile_t *s_file;    /* eject/unplug (fatrd_release), so an idle device pins none of it */
static int       s_ndir, s_nfile;
static unsigned  s_cap_dir, s_cap_file;   /* current allocation capacity (entries) */
static bool      s_of_dir, s_of_file;     /* a build pass overflowed dirs / files -> grow only that
                                           * dimension (doubling both wastes unused slots on a tight heap) */

/* Persistent cluster assignment (vpath -> first cluster). The model is rebuilt
 * from the VFS on every change, but each file must keep the same cluster across
 * rebuilds - otherwise the host's cached FAT (which it wrote against the clusters
 * we last showed it) desyncs, and reads/renames land on the wrong file. So a file
 * keeps its assigned cluster; only new/resized files draw fresh ones (from a
 * high-water mark, never recycled within a mount), and a host-created file records
 * the cluster the host itself chose. Reset on mount (fresh session view). */
typedef struct { char vpath[VPATH_MAX]; uint32_t start, nclus; } asn_t;
static asn_t    *s_asn;
static int       s_nasn;
static uint32_t  s_hiwater = 2;           /* next never-yet-used cluster this mount */
#define DATA_CLUSTERS (g_clusters)   /* usable cluster count (runtime) */

static const char *leaf(const char *vpath)
{ const char *s = strrchr(vpath, '/'); return s ? s + 1 : vpath; }

/* entries an item consumes in a directory: N LFN entries + 1 short entry */
static uint32_t name_entries(const char *name)
{
    uint32_t L = (uint32_t)strlen(name);
    return ((L + LFN_CHARS - 1) / LFN_CHARS) + 1;
}

/* ---- model: rebuilt from the VFS on every access ---- */

static void root_cb(const char *name, uint32_t size, bool is_dir, void *ctx)
{
    (void)ctx;
    if (is_dir) {
        if ((unsigned)s_ndir >= s_cap_dir) { s_of_dir = true; return; }
        snprintf(s_dir[s_ndir].vpath, VPATH_MAX, "/%s", name);
        s_dir[s_ndir].parent = -1;
        s_dir[s_ndir].is_ext = false;
        s_ndir++;
    } else {
        if ((unsigned)s_nfile >= s_cap_file) { s_of_file = true; return; }
        snprintf(s_file[s_nfile].vpath, VPATH_MAX, "/%s", name);
        s_file[s_nfile].size = size;
        s_file[s_nfile].parent = -1;
        s_nfile++;
    }
}

struct sub_ctx { int dir; const char *base; };
static void sub_cb(const char *name, uint32_t size, bool is_dir, void *vctx)
{
    struct sub_ctx *c = vctx;
    if (is_dir) {                            /* nested dir (e.g. /mnt/ext0): model it too */
        if ((unsigned)s_ndir >= s_cap_dir) { s_of_dir = true; return; }
        snprintf(s_dir[s_ndir].vpath, VPATH_MAX, "%s/%s", c->base, name);
        s_dir[s_ndir].parent = c->dir;
        s_dir[s_ndir].is_ext = false;
        s_ndir++;
    } else {
        if ((unsigned)s_nfile >= s_cap_file) { s_of_file = true; return; }
        snprintf(s_file[s_nfile].vpath, VPATH_MAX, "%s/%s", c->base, name);
        s_file[s_nfile].size = size;
        s_file[s_nfile].parent = c->dir;
        s_nfile++;
    }
}

/* number of entries in directory `dir` (-1 = root): leading specials + child
 * directories + child files (both keyed by parent). */
static uint32_t dir_entry_count(int dir)
{
    uint32_t n = (dir < 0) ? 1 : 2;          /* root: vol label; subdir: ".",".." */
    for (int d = 0; d < s_ndir; d++)
        if (s_dir[d].parent == dir) n += name_entries(leaf(s_dir[d].vpath));
    for (int i = 0; i < s_nfile; i++)
        if (s_file[i].parent == dir) n += name_entries(leaf(s_file[i].vpath));
    return n;
}

/* The synthetic model (dir/file lists + cluster layout) is derived from the live
 * filesystem. Rebuilding it is an O(files) VFS/LittleFS traversal - tens of ms
 * once the FS holds a few dozen files - so doing it on every 512-byte sector
 * access made a single mount's burst of reads slow enough to trip the host's I/O
 * timeout (intermittent "cannot mount", and outright hangs when the SoftDevice
 * radio also steals cycles). Instead, rebuild only when the FS actually changed:
 * every mutation bumps s_fs_gen (writes via fatrd_write below, everything else
 * via fatrd_invalidate()); build_model is a no-op while the model is current. */
static uint32_t s_fs_gen = 1;     /* incremented on every FS mutation */
static uint32_t s_built_gen;      /* generation the cached model reflects */
static bool     s_have_model;

/* FAT32 root is a cluster chain (not a fixed region); it gets a stable cluster
 * assignment like any directory - always cluster ROOT_CLUS (2) as the lowest. */
static uint32_t s_root_start = ROOT_CLUS;
static uint32_t s_root_nclus = 1;

/* When the filesystem changes out from under a host that has the MSC drive
 * mounted, the host keeps serving its cached directory view. Two signals fix that,
 * both driven from fatrd_invalidate() (see below): a "medium may have changed"
 * SCSI unit-attention on the next command (fatrd_media_changed ->
 * tud_msc_test_unit_ready_cb) for a desktop that polls the drive, and a monotonic
 * generation counter (s_ext_gen, exposed at GEN_LBA) the host CLI reads cache-free
 * to decide whether its mount is stale. Both are the standard removable-media
 * story - no out-of-band signalling. */
static volatile bool     s_media_dirty;
static volatile uint32_t s_ext_gen = 1;   /* bumped whenever a mounted host's view goes stale */

/* Every VFS mutation reaches here. A card-passthrough MSC write never does (it
 * touches the card's real sectors directly, so the host's mounted view already
 * matches). What does reach here is a write to the internal region - a proto/BLE
 * file op, or an internal-subtree MSC write whose data we reconcile out of the card
 * scratch into the VFS (which relocates the file to an internal cluster the host's
 * cache doesn't know). Both leave a mounted host's view stale, so both raise the
 * generation / media-changed signal that makes the host re-read. */
void fatrd_invalidate(void)
{
    s_fs_gen++;
    s_media_dirty = true;
    s_ext_gen++;
}

/* Storage critical section. The MSC model path (build_model + fill_sector's
 * vfs_pread) runs on the TinyUSB task; proto/VFS file ops run on the cli/proto
 * tasks. LittleFS is not reentrant, so an MSC read overlapping a proto write
 * corrupts the write (files land empty). This recursive mutex serialises every
 * LittleFS access. It is held only across lfs_* calls, never across a USB emit -
 * a proto reader releases it around each chunk it sends - so the TinyUSB task can
 * always drain its TX FIFO and the two never deadlock. Recursive because the MSC
 * write path re-enters via vfs_write_file while already holding it. */
static SemaphoreHandle_t s_store_lock;
void fatrd_store_init(void)   { if (!s_store_lock) s_store_lock = xSemaphoreCreateRecursiveMutex(); }
void fatrd_store_lock(void)   { if (s_store_lock) xSemaphoreTakeRecursive(s_store_lock, portMAX_DELAY); }
void fatrd_store_unlock(void) { if (s_store_lock) xSemaphoreGiveRecursive(s_store_lock); }

/* True (and self-clearing) when the FS changed since the host last checked. */
bool fatrd_media_changed(void)
{
    bool d = s_media_dirty;
    s_media_dirty = false;
    return d;
}

/* A reserved sector (unused by any host FAT structure) that reports the current
 * external-write generation. The host CLI reads this with an SG_IO READ - which
 * bypasses the page cache and always hits the device - so it can tell, race-free,
 * whether the filesystem changed under a mounted view since it last looked. Only
 * non-MSC mutations bump the count, so the host reading its own writes back does
 * not see a change. Magic "FSgen" (filesystem generation - this file is shared by
 * every platform, so the tag is device-neutral) guards a stray read from garbage. */
#define GEN_LBA 3u
static void synth_gen(uint8_t s[BPS])
{
    memset(s, 0, BPS);
    memcpy(s, "FSgen\0\0\0", 8);
    s[8] = (uint8_t)s_ext_gen; s[9] = (uint8_t)(s_ext_gen >> 8);
    s[10] = (uint8_t)(s_ext_gen >> 16); s[11] = (uint8_t)(s_ext_gen >> 24);
}

static bool model_ensure(unsigned nd, unsigned nf);   /* (re)allocate model buffers; defined after s_newdir */
static void tomb_flush(void);                         /* commit deferred deletes; defined near fatrd_read */
static void flush_pending(void);                      /* deferred-reconcile sweep; defined in the write section */
static void stage_free_all(void);                     /* drop all staged sectors; defined in the write section */
static uint32_t asn_assign(const char *vpath, uint32_t nclus);   /* stable cluster; defined after model_ensure */

/* True when `vpath` is the mount root of a real-FAT external device that we serve
 * by passthrough (so build_model must not walk it). */
static bool is_fat_ext_root(const char *vpath)
{
    fatrd_ext_t e;
    if (!fatrd_ext_present(&e)) return false;
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(vpath, &leaf);
    return m && vfs_mount_is_fat(m) && leaf && leaf[0] == '/' && leaf[1] == '\0';
}

static void build_model(void)
{
    if (s_have_model && s_built_gen == s_fs_gen) return;   /* cache still valid */

    /* Fill from the live FS; if a pass overflows the current capacity, double it
     * and refill. Converges in one or two passes and fits any FS that fits in heap
     * (model_ensure returns false on OOM -> callers present a blank drive). */
    unsigned cd = s_cap_dir ? s_cap_dir : INIT_DIRS;
    unsigned cf = s_cap_file ? s_cap_file : INIT_FILES;
    for (int attempt = 0; attempt < 16; attempt++) {
        if (!model_ensure(cd, cf)) return;
        s_ndir = 0; s_nfile = 0; s_of_dir = s_of_file = false;
        vfs_list("/", root_cb, NULL);   /* includes the synthetic /ramfs mount */
        for (int d = 0; d < s_ndir; d++) {
            /* A real-FAT external mount (the SD) is not synthesized - we don't walk
             * its (possibly huge) tree. It becomes a passthrough directory whose
             * cluster is the card's own root cluster; reads of it and everything
             * under it are served straight from the card. */
            if (is_fat_ext_root(s_dir[d].vpath)) { s_dir[d].is_ext = true; continue; }
            struct sub_ctx c = { d, s_dir[d].vpath };
            vfs_list(s_dir[d].vpath, sub_cb, &c);
        }
        if (!s_of_dir && !s_of_file) break;
        cd = s_of_dir  ? s_cap_dir  * 2 : s_cap_dir;   /* grow only the dimension that overflowed */
        cf = s_of_file ? s_cap_file * 2 : s_cap_file;
    }

    select_geometry();
    /* High-water = above every cluster still reserved by a prior assignment, not just
     * g_int_first. asn_assign hands a reused (stable) cluster back without advancing
     * the mark, so on a rebuild that mixes stable items (existing files) with a fresh
     * one (e.g. a file just added over proto, seen on a re-read of the still-mounted
     * drive) the fresh item would otherwise be handed a low cluster that collides with
     * a stable file's - the two then alias the same data. Starting above all reserved
     * clusters keeps fresh blocks strictly disjoint from reused ones. */
    s_hiwater = g_int_first;      /* internal clusters live above the passthrough window */
    for (int i = 0; i < s_nasn; i++) {
        uint32_t end = s_asn[i].start + s_asn[i].nclus;
        if (end > s_hiwater) s_hiwater = end;
    }

    /* Stable cluster layout: each item keeps the cluster it already holds; only
     * new/resized ones draw fresh blocks. The FAT32 root goes first so it lands on
     * the lowest internal cluster (g_int_first). Then snapshot the assignment so the
     * next rebuild (and rename detection) resolves the same clusters. */
    {
        uint32_t rents = dir_entry_count(-1);
        s_root_nclus = (rents + EPS - 1) / EPS + DIR_SLACK_CLUS;
        s_root_start = asn_assign("/", s_root_nclus);
    }
    for (int d = 0; d < s_ndir; d++) {
        if (s_dir[d].is_ext) {                 /* passthrough mount root -> card's root cluster */
            s_dir[d].start = g_ext_root_clus;
            s_dir[d].nclus = 0;                /* owns no synthetic clusters */
            continue;
        }
        uint32_t ents = dir_entry_count(d);
        s_dir[d].nclus = (ents + EPS - 1) / EPS + DIR_SLACK_CLUS;
        s_dir[d].start = asn_assign(s_dir[d].vpath, s_dir[d].nclus);
    }
    for (int i = 0; i < s_nfile; i++) {
        s_file[i].nclus = s_file[i].size ? (s_file[i].size + BPS - 1) / BPS : 1;
        s_file[i].start = asn_assign(s_file[i].vpath, s_file[i].nclus);
    }
    s_nasn = 0;
    if ((unsigned)s_nasn < s_cap_dir + s_cap_file) {   /* keep the root's cluster stable too */
        strcpy(s_asn[s_nasn].vpath, "/");
        s_asn[s_nasn].start = s_root_start; s_asn[s_nasn].nclus = s_root_nclus; s_nasn++;
    }
    for (int d = 0; d < s_ndir && (unsigned)s_nasn < s_cap_dir + s_cap_file; d++) {
        strcpy(s_asn[s_nasn].vpath, s_dir[d].vpath);
        s_asn[s_nasn].start = s_dir[d].start; s_asn[s_nasn].nclus = s_dir[d].nclus; s_nasn++;
    }
    for (int i = 0; i < s_nfile && (unsigned)s_nasn < s_cap_dir + s_cap_file; i++) {
        strcpy(s_asn[s_nasn].vpath, s_file[i].vpath);
        s_asn[s_nasn].start = s_file[i].start; s_asn[s_nasn].nclus = s_file[i].nclus; s_nasn++;
    }
    s_built_gen = s_fs_gen;
    s_have_model = true;
}

/* Which directory owns cluster c? -1 means "the root" when c is in the root's
 * chain, or "none" otherwise - the two are disambiguated by cluster_is_root(). */
static bool cluster_is_root(uint32_t c)
{
    return c >= s_root_start && c < s_root_start + s_root_nclus;
}
static int dir_of_cluster(uint32_t c)
{
    for (int d = 0; d < s_ndir; d++)
        if (c >= s_dir[d].start && c < s_dir[d].start + s_dir[d].nclus) return d;
    return -1;
}
static synfile_t *file_of_cluster(uint32_t c)
{
    for (int i = 0; i < s_nfile; i++)
        if (c >= s_file[i].start && c < s_file[i].start + s_file[i].nclus) return &s_file[i];
    return NULL;
}

/* When the host creates a subdirectory mid-mount it allocates a cluster for it
 * out of the FAT free list - a cluster our model will assign elsewhere on the
 * next rebuild. We remember that host-chosen cluster here so entry writes the
 * host makes into the new directory (e.g. a file created inside it before the
 * next remount) are routed to the right directory instead of being mistaken for
 * raw file data. Cleared on every mount (boot-sector read) so a stale mapping
 * can't misroute a later session's writes. */
#define MAX_NEWDIR 8
static struct { uint32_t cluster; char vpath[VPATH_MAX]; } *s_newdir;
static int s_nnewdir;

/* Host-allocated clusters that extend a directory past the model's contiguous
 * range: when a directory's presented clusters fill up, the host chains a fresh
 * free cluster onto the tail and writes further entries there. The synthetic FAT
 * doesn't store the host's links, but we learn this one from its FAT write and
 * remember cluster -> owning directory, so the entry writes that follow route to
 * write_dir instead of being captured as raw file data (and silently lost).
 * dir: -1 = root, >=0 = s_dir index. Cleared each mount (boot-sector read). */
#define MAX_DIREXT 12
static struct dirext_s { uint32_t cluster; int16_t dir; } *s_dirext;   /* lazy heap; see recon_bufs_* */
static int s_ndirext;

/* Deferred reconcile: the host can write a file's directory entry before all of its
 * data has been staged, so flushing inline at the entry write would miss. Instead a
 * not-yet-complete file is recorded here (metadata only - the data stays in the one
 * staging list, no extra copy) and flushed the moment its last sector stages, with a
 * sweep on the next read as a backstop. Order between data and entry stops mattering.
 * Cleared each mount. If the table fills, the overflow falls back to an inline flush. */
#define MAX_PENDING 16
static struct pending_s { char vpath[VPATH_MAX]; uint32_t start, size; bool used; } *s_pending;   /* lazy heap */

/* The host's cluster chains, learned from its internal FAT writes. A file allocated
 * contiguously needs no entry - the default next cluster is clus+1 - so this stores
 * only the non-contiguous jumps. It lets do_flush reassemble a file the host
 * fragmented (interleaving several files' cluster allocations in one copy) from its
 * real chain instead of reading straight through into a neighbour's data. Cleared
 * each mount; on overflow a file falls back to the contiguous walk. */
#define MAX_FATCHAIN 96
static struct fatchain_s { uint32_t clus, next; } *s_fatchain;   /* lazy heap */
static int s_nfatchain;
static uint32_t chain_next(uint32_t c)
{
    for (int i = 0; i < s_nfatchain; i++) if (s_fatchain[i].clus == c) return s_fatchain[i].next;
    return c + 1;   /* no recorded jump: contiguous (also the not-allocated case, s_nfatchain == 0) */
}
static void chain_record(uint32_t c, uint32_t next)
{
    if (!s_fatchain) return;
    for (int i = 0; i < s_nfatchain; i++) if (s_fatchain[i].clus == c) { s_fatchain[i].next = next; return; }
    if (s_nfatchain < MAX_FATCHAIN) { s_fatchain[s_nfatchain].clus = c; s_fatchain[s_nfatchain].next = next; s_nfatchain++; }
}

/* Owning directory of cluster c across the model chains and host extensions:
 * -1 = root, >=0 = s_dir index, -2 = not directory space. */
static int dir_owner(uint32_t c)
{
    if (cluster_is_root(c)) return -1;
    int d = dir_of_cluster(c);
    if (d >= 0) return d;
    for (int i = 0; i < s_ndirext; i++) if (s_dirext[i].cluster == c) return s_dirext[i].dir;
    return -2;
}

/* Partial long-name reconstruction, keyed by the LFN checksum that every fragment
 * and its short entry share (a byte-sum over the 8.3 name). A file's LFN fragments
 * and its short entry can land in different clusters, written in separate MSC
 * transfers with other directory writes interleaved between - keying each partial
 * name by checksum keeps one file's fragments from overwriting another's. A single
 * shared buffer collapsed a straddling long name to its 8.3 form. Cleared per mount. */
#define LFN_POOL 16
static struct lfnp_s { uint8_t cksum; bool used; char name[64]; } *s_lfnp;   /* lazy heap */
static int lfn_find(uint8_t ck)
{ if (!s_lfnp) return -1; for (int i = 0; i < LFN_POOL; i++) if (s_lfnp[i].used && s_lfnp[i].cksum == ck) return i; return -1; }
static int lfn_slot(uint8_t ck)
{
    if (!s_lfnp) return -1;
    int i = lfn_find(ck); if (i >= 0) return i;
    for (i = 0; i < LFN_POOL; i++) if (!s_lfnp[i].used) {
        s_lfnp[i].used = true; s_lfnp[i].cksum = ck; memset(s_lfnp[i].name, 0, sizeof s_lfnp[i].name); return i;
    }
    return -1;   /* pool full: this name falls back to 8.3 */
}
static uint8_t sfn_cksum(const uint8_t *e)
{ uint8_t s = 0; for (int i = 0; i < 11; i++) s = (uint8_t)(((s & 1) << 7) + (s >> 1) + e[i]); return s; }

/* The model (s_dir/s_file/s_newdir, ~6 KB) is heap-resident only while a host is actually using the MSC
 * drive. Allocated lazily on the first read/write (via build_model), freed by fatrd_release() on eject /
 * USB unplug. All of it is touched only on the usb task (tud_msc_* callbacks run from tud_task), so no
 * locking; fatrd_invalidate (other tasks) bumps s_fs_gen only, never the buffers. */
/* Ensure the model buffers hold at least `nd` dirs and `nf` files. Reallocates
 * (free + malloc; the caller refills from scratch, so no copy needed) when the
 * request exceeds the current capacity. Returns false on OOM. */
static bool model_ensure(unsigned nd, unsigned nf)
{
    if (!s_newdir) s_newdir = pvPortMalloc(MAX_NEWDIR * sizeof *s_newdir);
    if (s_file && nd <= s_cap_dir && nf <= s_cap_file && s_newdir) return true;
    if (!s_newdir) return false;

    /* s_dir/s_file are refilled from scratch, so free them before allocating the
     * replacements: holding old + new at once nearly doubles the transient peak, which
     * a tight heap can't spare. Only s_asn (the persistent cluster assignment) must
     * survive, so it alone is migrated with a copy. */
    vPortFree(s_dir);  s_dir  = NULL;
    vPortFree(s_file); s_file = NULL;
    s_cap_dir = s_cap_file = 0;

    asn_t *nasn = pvPortMalloc((nd + nf) * sizeof *nasn);
    if (nasn) {                                    /* migrate the assignment, drop the old copy */
        if (s_asn && s_nasn > 0) memcpy(nasn, s_asn, s_nasn * sizeof *nasn);
        vPortFree(s_asn);
        s_asn = nasn;
    }
    syndir_t  *ndir  = pvPortMalloc(nd * sizeof *ndir);
    synfile_t *nfile = pvPortMalloc(nf * sizeof *nfile);
    if (!nasn || !ndir || !nfile) {                /* OOM: drop it all, caller shows a blank drive */
        vPortFree(ndir); vPortFree(nfile);
        vPortFree(s_asn); s_asn = NULL; s_nasn = 0;
        return false;
    }
    s_dir = ndir; s_file = nfile;
    s_cap_dir = nd; s_cap_file = nf;
    return true;
}

/* Renamed-away / deleted files awaiting commit (see the tomb_* section below). Kept
 * here with the other lazy-heap reconcile buffers so recon_bufs_* can manage it. */
#define MAX_TOMB 16
static struct tomb_s { char vpath[VPATH_MAX]; uint32_t cluster; } *s_tomb;   /* lazy heap */
static int s_ntomb;

/* The write-side reconcile buffers (~5 KB) are only needed while a host is writing the
 * MSC drive, so keep them off the static heap: allocate all-or-nothing on the first
 * captured write and free them on eject/unplug (via fatrd_release), leaving an idle
 * device the full heap. A single sentinel pointer (s_pending) gates the set; count vars
 * stay zero while freed, so the count-based scans (tomb/fatchain/dirext) no-op and only
 * the .used-flag scans need a guard. */
static bool recon_bufs_ensure(void)
{
    if (s_pending) return true;                          /* already resident for this session */
    void *pd = pvPortMalloc(MAX_PENDING  * sizeof *s_pending);
    void *tb = pvPortMalloc(MAX_TOMB     * sizeof *s_tomb);
    void *fc = pvPortMalloc(MAX_FATCHAIN * sizeof *s_fatchain);
    void *lf = pvPortMalloc(LFN_POOL     * sizeof *s_lfnp);
    void *dx = pvPortMalloc(MAX_DIREXT   * sizeof *s_dirext);
    if (!pd || !tb || !fc || !lf || !dx) {               /* OOM: keep none, caller fails the write */
        vPortFree(pd); vPortFree(tb); vPortFree(fc); vPortFree(lf); vPortFree(dx);
        return false;
    }
    s_pending = pd; s_tomb = tb; s_fatchain = fc; s_lfnp = lf; s_dirext = dx;
    memset(s_pending, 0, MAX_PENDING * sizeof *s_pending);   /* .used = false */
    memset(s_lfnp,    0, LFN_POOL    * sizeof *s_lfnp);      /* .used = false */
    s_ntomb = s_nfatchain = s_ndirext = 0;
    return true;
}
static void recon_bufs_free(void)
{
    vPortFree(s_pending); vPortFree(s_tomb); vPortFree(s_fatchain);
    vPortFree(s_lfnp);    vPortFree(s_dirext);
    s_pending = NULL; s_tomb = NULL; s_fatchain = NULL; s_lfnp = NULL; s_dirext = NULL;
    s_ntomb = s_nfatchain = s_ndirext = 0;
}

void fatrd_release(void)
{
    tomb_flush();                                        /* commit any pending deletes (no-op if none) */
    vPortFree(s_dir); vPortFree(s_file); vPortFree(s_newdir); vPortFree(s_asn);
    s_dir = NULL; s_file = NULL; s_newdir = NULL; s_asn = NULL;
    s_cap_dir = s_cap_file = 0; s_nasn = 0; s_hiwater = 2;
    s_have_model = false; s_nnewdir = 0;                 /* next access rebuilds from scratch */
    stage_free_all();
    recon_bufs_free();                                   /* hand the ~5 KB back to the idle-time heap */
}

/* Reuse the cluster this vpath already holds (same size), else draw a fresh block
 * from the high-water mark - never a recycled one, so the host's cached view of
 * live files stays valid. Returns 0 when the drive is out of clusters. */
static uint32_t asn_assign(const char *vpath, uint32_t nclus)
{
    for (int i = 0; i < s_nasn; i++)
        if (s_asn[i].nclus == nclus && strcmp(s_asn[i].vpath, vpath) == 0)
            return s_asn[i].start;
    if (s_hiwater + nclus > DATA_CLUSTERS + 2) return 0;   /* disk full */
    uint32_t start = s_hiwater;
    s_hiwater += nclus;
    return start;
}

/* Record/adopt a specific assignment for a host-created file (its own cluster
 * choice), so the next rebuild presents the file exactly where the host put it. */
static void asn_put(const char *vpath, uint32_t start, uint32_t nclus)
{
    for (int i = 0; i < s_nasn; i++)
        if (strcmp(s_asn[i].vpath, vpath) == 0) {
            s_asn[i].start = start; s_asn[i].nclus = nclus; return;
        }
    if ((unsigned)s_nasn < s_cap_dir + s_cap_file) {
        strncpy(s_asn[s_nasn].vpath, vpath, VPATH_MAX - 1);
        s_asn[s_nasn].vpath[VPATH_MAX - 1] = '\0';
        s_asn[s_nasn].start = start; s_asn[s_nasn].nclus = nclus; s_nasn++;
    }
}

/* Carry an assignment across a rename so the destination keeps the source's
 * cluster (the host kept the data in place, only the name changed). */
static void asn_rename(const char *from, const char *to)
{
    for (int i = 0; i < s_nasn; i++)
        if (strcmp(s_asn[i].vpath, from) == 0) {
            strncpy(s_asn[i].vpath, to, VPATH_MAX - 1);
            s_asn[i].vpath[VPATH_MAX - 1] = '\0';
            return;
        }
}
static const char *newdir_lookup(uint32_t clus)
{
    for (int i = 0; i < s_nnewdir; i++) if (s_newdir[i].cluster == clus) return s_newdir[i].vpath;
    return NULL;
}
static int dir_index(const char *vpath)
{
    for (int d = 0; d < s_ndir; d++) if (strcmp(s_dir[d].vpath, vpath) == 0) return d;
    return -1;
}

/* ---- LE writers ---- */
static void put16(uint8_t *p, uint16_t v) { p[0]=v; p[1]=v>>8; }
static void put32(uint8_t *p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }

/* Deterministic, unique-within-dir 8.3 short name for an item; the LFN carries
 * the real name. idx is the item's position in its directory. */
static void short_name(int idx, const char *name, uint8_t out[11])
{
    memset(out, ' ', 11);
    char base[16];
    snprintf(base, sizeof(base), "FAP%05d", idx & 0xfffff);   /* 8 chars, valid+unique */
    memcpy(out, base, 8);
    const char *dot = strrchr(name, '.');
    if (dot) for (int i = 0; i < 3 && dot[1+i]; i++) {
        char ch = dot[1+i];
        out[8+i] = (ch>='a'&&ch<='z') ? ch-32 : ch;
    }
}

static uint8_t sfn_checksum(const uint8_t s[11])
{
    uint8_t sum = 0;
    for (int i = 0; i < 11; i++) sum = (uint8_t)(((sum & 1) << 7) + (sum >> 1) + s[i]);
    return sum;
}

static void short_entry(uint8_t *e, const uint8_t nm[11], uint8_t attr, uint32_t start, uint32_t size)
{
    memset(e, 0, 32);
    memcpy(e, nm, 11);
    e[11] = attr;
    put16(e + 20, (uint16_t)(start >> 16));   /* FAT32: high word of first cluster */
    put16(e + 26, (uint16_t)start);           /* low word */
    put32(e + 28, size);
}

/* Fill one LFN entry: seq (1-based, |0x40 on the last/highest), holding the
 * 13-char window of `name` starting at (seq-1)*13. */
static void lfn_entry(uint8_t *e, const char *name, int seq, bool last, uint8_t cksum)
{
    memset(e, 0, 32);
    e[0] = (uint8_t)(seq | (last ? 0x40 : 0));
    e[11] = 0x0F;        /* LFN attr */
    e[13] = cksum;
    static const int pos[13] = {1,3,5,7,9, 14,16,18,20,22,24, 28,30};
    int base = (seq - 1) * LFN_CHARS;
    int L = (int)strlen(name);
    for (int i = 0; i < LFN_CHARS; i++) {
        int ci = base + i;
        uint16_t ch;
        if (ci < L) ch = (uint8_t)name[ci];
        else if (ci == L) ch = 0x0000;       /* NUL terminator */
        else ch = 0xFFFF;                     /* padding */
        e[pos[i]] = ch & 0xff; e[pos[i]+1] = ch >> 8;
    }
}

/* ---- READ (synthesis) ---- */

static void synth_boot(uint8_t s[BPS])
{
    memset(s, 0, BPS);
    s[0]=0xEB; s[1]=0x58; s[2]=0x90; memcpy(s+3,"MSDOS5.0",8);
    put16(s+11,BPS); s[13]=(uint8_t)g_sec_per_clus; put16(s+14,RSVD); s[16]=NFAT;
    put16(s+17, 0); put16(s+19, 0);          /* FAT32: root-ent-cnt=0, tot16=0 */
    s[21]=0xF8;
    put16(s+22, 0);                          /* FATSz16 = 0 (FAT32) */
    put16(s+24, 63); put16(s+26, 255);       /* sec/track, heads (cosmetic) */
    put32(s+32, g_total_sec);                /* tot32 */
    /* FAT32 extended BPB */
    put32(s+36, g_fatsz);                    /* FATSz32 */
    put16(s+40, 0);                          /* ext flags: FAT0 active, mirrored */
    put16(s+42, 0);                          /* FS version 0.0 */
    put32(s+44, s_root_start);               /* root dir cluster: the merged synthetic
                                              * root (internal range), not the card's
                                              * cluster 2 which lives in the passthrough
                                              * window and holds the card's own root */
    put16(s+48, FSINFO_LBA);                 /* FSInfo sector */
    put16(s+50, 6);                          /* backup boot sector */
    s[64]=0x80; s[66]=0x29; put32(s+67,0xFA17A511);
    memcpy(s+71,"Fantasi    ",11); memcpy(s+82,"FAT32   ",8);
    s[510]=0x55; s[511]=0xAA;
}

static void synth_fsinfo(uint8_t s[BPS])
{
    /* Report a valid free count so the host does not scan the whole FAT to compute
     * it - on a card-sized volume that scan is thousands of slow passthrough sector
     * reads (which made every mount/stat crawl). With a card, all free space is the
     * card's (the internal region is deliberately full); without one, it's the
     * internal region's spare clusters. The next-free hint points at the card so
     * host allocations land there (see the write reconcile). */
    uint32_t free_clst = g_pt_max ? g_card_free
                       : (g_clusters > (s_hiwater - 2u) ? g_clusters - (s_hiwater - 2u) : 0);
    memset(s, 0, BPS);
    put32(s+0,   0x41615252);                /* lead signature */
    put32(s+484, 0x61417272);                /* struct signature */
    put32(s+488, free_clst);                 /* free cluster count */
    put32(s+492, g_pt_max ? 2u : s_hiwater); /* next-free hint */
    s[510]=0x55; s[511]=0xAA;
}

/* 32-bit FAT entry for cluster e: next-in-chain, EOC, or 0 (free). Chains cover
 * the root, every subdirectory, and every file. */
static uint32_t fat_entry(uint32_t e)
{
    if (e == 0) return 0x0FFFFFF8;
    if (e == 1) return 0x0FFFFFFF;
    if (cluster_is_root(e))
        return (e == s_root_start + s_root_nclus - 1) ? 0x0FFFFFFF : e + 1;
    for (int d = 0; d < s_ndir; d++)
        if (e >= s_dir[d].start && e < s_dir[d].start + s_dir[d].nclus)
            return (e == s_dir[d].start + s_dir[d].nclus - 1) ? 0x0FFFFFFF : e + 1;
    synfile_t *f = file_of_cluster(e);
    if (f) return (e == f->start + f->nclus - 1) ? 0x0FFFFFFF : e + 1;
    /* When a card is present, the internal region must expose NO free clusters, so
     * the host allocates only real card clusters (which passthrough handles); an
     * internal-subtree write's data then lands in the card scratch and is
     * reconciled into the VFS. An unreferenced internal cluster reads as a reserved
     * "used" marker rather than free. Internal-only (no card) keeps free space so
     * the drive is directly writable. */
    if (g_pt_max && e >= g_int_first) return 0x0FFFFFFF;
    return 0x00000000;
}

static void synth_fat(uint32_t sec, uint8_t s[BPS])
{
    uint32_t first = sec * (BPS/4);          /* 128 32-bit entries per sector */
    for (uint32_t i = 0; i < BPS/4; i++) put32(s + i*4, fat_entry(first + i));
}

/* Item lookup within a directory: returns the n-th item (0-based) as name/attr/
 * start/size, or NULL when out of range. idx_out gets the item index in the dir. */
static const char *dir_item(int dir, int n, uint8_t *attr, uint32_t *start, uint32_t *size, int *idx_out)
{
    int k = 0;
    for (int d = 0; d < s_ndir; d++) if (s_dir[d].parent == dir) {   /* child directories first */
        if (k == n) { *attr=0x10; *start=s_dir[d].start; *size=0; *idx_out=k; return leaf(s_dir[d].vpath); }
        k++;
    }
    for (int i = 0; i < s_nfile; i++) if (s_file[i].parent == dir) { /* then child files */
        if (k == n) { *attr=0x20; *start=s_file[i].start; *size=s_file[i].size; *idx_out=k; return leaf(s_file[i].vpath); }
        k++;
    }
    return NULL;
}

/* Fill the 32-byte directory entry at global index g for directory `dir`. */
static void emit_entry(int dir, uint32_t g, uint8_t *e)
{
    memset(e, 0, 32);
    uint32_t lead = (dir < 0) ? 1 : 2;
    if (dir < 0 && g == 0) {                 /* root volume label */
        uint8_t nm[11]; memcpy(nm, "Fantasi    ", 11);
        short_entry(e, nm, 0x08, 0, 0);
        return;
    }
    if (dir >= 0 && g == 0) { uint8_t nm[11]; memset(nm,' ',11); nm[0]='.'; short_entry(e, nm, 0x10, s_dir[dir].start, 0); return; }
    if (dir >= 0 && g == 1) {                /* ".." points to the parent dir (0 = root) */
        int par = s_dir[dir].parent;
        uint32_t pstart = (par < 0) ? 0 : s_dir[par].start;
        uint8_t nm[11]; memset(nm,' ',11); nm[0]='.'; nm[1]='.'; short_entry(e, nm, 0x10, pstart, 0); return;
    }

    uint32_t pos = lead;
    for (int n = 0; ; n++) {
        uint8_t attr; uint32_t start, size; int idx;
        const char *name = dir_item(dir, n, &attr, &start, &size, &idx);
        if (!name) return;                   /* past the last item -> free entry */
        uint32_t k = name_entries(name);     /* LFN count + 1 */
        if (g < pos + k) {
            uint8_t sn[11]; short_name(idx, name, sn);
            uint32_t off = g - pos;          /* 0..k-1 */
            if (off < k - 1) {
                int total_lfn = (int)(k - 1);
                int seq = total_lfn - (int)off; /* highest seq first */
                lfn_entry(e, name, seq, off == 0, sfn_checksum(sn));
            } else {
                short_entry(e, sn, attr, start, size);
            }
            return;
        }
        pos += k;
    }
}

static void synth_dir(int dir, uint32_t first_entry, uint8_t s[BPS])
{
    memset(s, 0, BPS);
    for (uint32_t i = 0; i < EPS; i++) emit_entry(dir, first_entry + i, s + i*32);
}

/* ---- Deferred deletes ("tombstones") ----
 * A directory 0xE5 means the name is unused, but that is either a delete or the
 * old half of a rename (whose new entry, pointing at the same first cluster, may
 * land in a later sector = later write). So a 0xE5 records a tombstone {vpath,
 * first-cluster} instead of removing immediately. A subsequent new entry that
 * reuses the cluster claims the tombstone as a rename; anything left over is a
 * real delete, committed the next time the host *reads* a directory (tomb_flush,
 * before synthesis) - so a reader never sees the intermediate state. The vpath,
 * not the cluster, is what gets removed, so a stale cluster can't hit the wrong
 * file; the cluster is only used to match a rename within one stable-model op. */
/* s_tomb / MAX_TOMB are declared up with the other lazy-heap reconcile buffers. */

static void tomb_add(const char *vpath, uint32_t cluster)
{
    if (!s_tomb) return;
    if (s_ntomb >= MAX_TOMB) {            /* commit the oldest to make room */
        vfs_remove(s_tomb[0].vpath);
        memmove(s_tomb, s_tomb + 1, (MAX_TOMB - 1) * sizeof s_tomb[0]);
        s_ntomb = MAX_TOMB - 1;
    }
    strncpy(s_tomb[s_ntomb].vpath, vpath, VPATH_MAX - 1);
    s_tomb[s_ntomb].vpath[VPATH_MAX - 1] = '\0';
    s_tomb[s_ntomb].cluster = cluster;
    s_ntomb++;
}

/* If a tombstone owns `cluster`, copy its vpath to `out` and drop it (a rename is
 * reclaiming that file's data under a new name). Returns true on a match. */
static bool tomb_claim(uint32_t cluster, char *out)
{
    for (int i = 0; i < s_ntomb; i++)
        if (s_tomb[i].cluster == cluster) {
            strcpy(out, s_tomb[i].vpath);
            memmove(s_tomb + i, s_tomb + i + 1, (s_ntomb - i - 1) * sizeof s_tomb[0]);
            s_ntomb--;
            return true;
        }
    return false;
}

static void tomb_flush(void)             /* commit the surviving deletes */
{
    for (int i = 0; i < s_ntomb; i++) vfs_remove(s_tomb[i].vpath);
    s_ntomb = 0;
}

/* Synthesize one 512-byte sector `lba` into `sec` (assumes the model is built). */
static void fill_sector(uint32_t lba, uint8_t sec[BPS])
{
    if (lba == 0 || lba == 6) synth_boot(sec);            /* boot + its backup */
    else if (lba == FSINFO_LBA) synth_fsinfo(sec);
    else if (lba < g_fat_lba) memset(sec, 0, BPS);        /* reserved slack */
    else if (lba < g_fat_lba + NFAT * g_fatsz) {
        /* FAT sector: entries for card clusters mirror the card's FAT verbatim (its
         * cluster numbers match ours in the window), entries for internal clusters
         * are synthesized. A pure-card sector is passed straight through; the single
         * boundary sector reads the card FAT then overwrites its internal entries. */
        uint32_t fsec  = (lba - g_fat_lba) % g_fatsz;
        uint32_t first = fsec * 128u;
        if (g_pt_max && first + 127u <= g_pt_max) {
            if (fatrd_ext_read(g_pt_fat_lba + fsec, sec) != 0) memset(sec, 0, BPS);
        } else if (g_pt_max && first <= g_pt_max) {
            if (fatrd_ext_read(g_pt_fat_lba + fsec, sec) != 0) memset(sec, 0, BPS);
            for (uint32_t i = 0; i < 128u; i++)
                if (first + i > g_pt_max) put32(sec + i*4, fat_entry(first + i));
        } else {
            synth_fat(fsec, sec);
        }
    }
    else {
        uint32_t rel   = lba - g_data_lba;
        uint32_t clus  = rel / g_sec_per_clus + 2u;
        uint32_t intra = rel % g_sec_per_clus;            /* sector within the cluster */
        int d;
        if (cluster_is_passthrough(clus)) {               /* card data/dir sector, raw */
            if (fatrd_ext_read(g_pt_data_lba + (clus - 2u) * g_sec_per_clus + intra, sec) != 0)
                memset(sec, 0, BPS);
        }
        else if (cluster_is_root(clus))
            synth_dir(-1, ((clus - s_root_start) * g_sec_per_clus + intra) * EPS, sec);
        else if ((d = dir_of_cluster(clus)) >= 0)
            synth_dir(d, ((clus - s_dir[d].start) * g_sec_per_clus + intra) * EPS, sec);
        else {
            memset(sec, 0, BPS);
            synfile_t *f = file_of_cluster(clus);
            if (f) vfs_pread(f->vpath,
                             ((clus - f->start) * g_sec_per_clus + intra) * BPS, sec, BPS);
        }
    }
}

/* If merged sector `mlba` is served by raw card passthrough (a data/dir sector or
 * a pure-card FAT sector), return its card LBA; else 0. Consecutive passthrough
 * sectors map to consecutive card LBAs, so a run can be read in one multi-block
 * transfer - the difference between a mount reading the FAT sector-by-sector (slow)
 * and in big bursts. */
static uint32_t passthrough_lba(uint32_t mlba)
{
    if (!g_pt_max) return 0;
    if (mlba >= g_fat_lba && mlba < g_fat_lba + NFAT * g_fatsz) {
        uint32_t fsec = (mlba - g_fat_lba) % g_fatsz;
        return (fsec * 128u + 127u <= g_pt_max) ? g_pt_fat_lba + fsec : 0;   /* pure-card FAT */
    }
    if (mlba >= g_data_lba) {
        uint32_t rel  = mlba - g_data_lba;
        uint32_t clus = rel / g_sec_per_clus + 2u;
        return (clus <= g_pt_max) ? g_pt_data_lba + rel : 0;                 /* card data/dir */
    }
    return 0;
}

int fatrd_read(uint32_t lba, uint32_t offset, void *buf, uint32_t len)
{
    /* The generation sector is answered directly - no model rebuild - so the host's
     * pre-read freshness probe is a cheap single-sector read. */
    if (lba == GEN_LBA) {
        uint8_t sec[BPS]; synth_gen(sec);
        if (offset >= BPS) return -1;
        if (offset + len > BPS) len = BPS - offset;
        memcpy(buf, sec + offset, len);
        return 0;
    }
    /* Hold the storage lock across the whole model build + sector fill: build_model
     * and fill_sector's vfs_pread both walk LittleFS, which a proto write must not
     * touch concurrently. fatrd_read never emits USB, so holding it here can't stall
     * the TX drain. */
    fatrd_store_lock();
    /* A fresh-mount boot-sector read starts a new session. First backstop the deferred
     * reconcile - flush any file the host wrote but never SYNCHRONIZE CACHE'd (fatrd_sync
     * is the normal trigger) - then drop stale routes/assignments and free whatever
     * staging is left (abandoned partials). Non-boot reads don't reconcile: the host
     * reads its own cache mid-write, so committing partial state early would just churn. */
    if (lba == 0) { flush_pending();
                    s_nnewdir = 0; s_ndirext = 0; s_nfatchain = 0; s_nasn = 0; s_hiwater = 2; s_have_model = false;
                    if (s_lfnp)    for (int k = 0; k < LFN_POOL; k++)   s_lfnp[k].used = false;
                    if (s_pending) for (int k = 0; k < MAX_PENDING; k++) s_pending[k].used = false;
                    stage_free_all(); }
    /* Any read may observe a directory listing, so commit deferred deletes first;
     * a rename writes its 0xE5 + new entry back-to-back with no read between, so
     * this never pre-empts an in-flight rename. */
    tomb_flush();
    build_model();
    if (!s_file) { memset(buf, 0, len); fatrd_store_unlock(); return 0; }   /* model couldn't allocate (OOM): present a blank sector, don't crash */

    /* The transfer may span several sectors (CFG_TUD_MSC_EP_BUFSIZE > 512). Fill it
     * sector by sector, but batch a run of consecutive card-passthrough sectors into
     * one multi-block read straight into the caller's buffer. `offset` is only ever
     * nonzero when the EP buffer is smaller than a sector - handle that sector alone. */
    uint8_t *out = (uint8_t *)buf;
    uint32_t remaining = len, off = offset, cur = lba;

    while (remaining) {
        if (cur >= g_total_sec) { memset(out, 0, remaining); break; }

        uint32_t clba;
        if (off == 0 && remaining >= BPS && (clba = passthrough_lba(cur)) != 0) {
            uint32_t k = 1, want = remaining / BPS;
            while (k < want && passthrough_lba(cur + k) == clba + k) k++;
            if (fatrd_ext_read_multi(clba, out, k) != 0) memset(out, 0, k * BPS);
            out += k * BPS; remaining -= k * BPS; cur += k;
        } else {
            uint8_t sec[BPS];
            fill_sector(cur, sec);
            uint32_t n = BPS - off; if (n > remaining) n = remaining;
            memcpy(out, sec + off, n);
            out += n; remaining -= n; off = 0; cur++;
        }
    }
    fatrd_store_unlock();
    return 0;
}

/* ---- WRITE (capture) ---- */

// Heap held in reserve while staging a host write
#define STAGE_MIN_FREE 6144u

typedef struct stage { struct stage *next; uint32_t cluster; uint8_t data[BPS]; } stage_t;
static stage_t *s_stage;
static stage_t *stage_find(uint32_t c)
{ for (stage_t *p = s_stage; p; p = p->next) if (p->cluster == c) return p; return NULL; }
static void stage_free_all(void)
{ while (s_stage) { stage_t *n = s_stage->next; vPortFree(s_stage); s_stage = n; } }

/* True once every data cluster of the file has been staged, walking the host's real
 * cluster chain (chain_next) rather than assuming contiguous clusters. */
static bool all_staged(uint32_t start, uint32_t size)
{
    uint32_t nclus = size ? (size + BPS - 1) / BPS : 0, clus = start;
    for (uint32_t k = 0; k < nclus; k++) {
        if (clus < 2 || !stage_find(clus)) return false;
        clus = chain_next(clus);
    }
    return true;
}

/* Write a fully-staged file into the VFS along its cluster chain, streaming one
 * sector at a time (write the first, append the rest) and freeing each staged
 * sector as it goes. Peak heap is one sector, not the whole file - a large file
 * (e.g. 13 KB) can't be malloc'd whole on a small heap, and the data already lives
 * in the staging list, so buffering it again would just double the footprint.
 * Assumes all_staged(start,size). */
static void do_flush(const char *vpath, uint32_t start, uint32_t size)
{
    uint32_t nclus = size ? (size + BPS - 1) / BPS : 0, clus = start, off = 0;
    for (uint32_t k = 0; k < nclus; k++) {
        stage_t *st = NULL;   /* unlink each sector as consumed - reconcile runs once, at a
                               * quiescent point with the final size, so nothing re-reads it. */
        for (stage_t **pp = &s_stage; *pp; pp = &(*pp)->next)
            if ((*pp)->cluster == clus) { st = *pp; *pp = st->next; break; }
        uint32_t n = size - off; if (n > BPS) n = BPS;
        if (st) { vfs_pwrite(vpath, off, st->data, n); vPortFree(st); }   /* positional: one sector of RAM */
        off += n;
        clus = chain_next(clus);
    }
    /* Set the exact length. Needed when the host overwrites a file with a shorter one:
     * the positional writes above replace the head but leave the old tail; this drops
     * it. (Nothing zeroes the freed bytes - they're just no longer part of the file.) */
    vfs_truncate(vpath, size);
}

/* Record a file the host named in a directory entry, keyed by its first cluster.
 * The size only ever grows here (the host rewrites the entry as the file grows), so
 * we track the maximum seen; reconcile flushes at that size once the data is present. */
static void pending_add(const char *vpath, uint32_t start, uint32_t size)
{
    if (!s_pending) return;
    for (int i = 0; i < MAX_PENDING; i++)
        if (s_pending[i].used && s_pending[i].start == start) {
            snprintf(s_pending[i].vpath, VPATH_MAX, "%s", vpath);
            if (size > s_pending[i].size) s_pending[i].size = size;
            return;
        }
    for (int i = 0; i < MAX_PENDING; i++)
        if (!s_pending[i].used) {
            s_pending[i].used = true; s_pending[i].start = start; s_pending[i].size = size;
            snprintf(s_pending[i].vpath, VPATH_MAX, "%s", vpath); return;
        }
}

/* Reconcile: commit every recorded file whose data (following its cluster chain) is
 * fully staged, then retire the record and its staging. Called only at quiescent
 * points - the host's SCSI sync and a fresh mount - so the recorded size is final and
 * one pass per file is enough; a still-incomplete file stays recorded for next time. */
static void flush_pending(void)
{
    if (!s_pending) return;                              /* no writes captured this session */
    for (int i = 0; i < MAX_PENDING; i++)
        if (s_pending[i].used && all_staged(s_pending[i].start, s_pending[i].size)) {
            do_flush(s_pending[i].vpath, s_pending[i].start, s_pending[i].size);
            s_pending[i].used = false;
        }
}

/* A directory entry named this file. Only record it (path, first cluster, size) -
 * the actual write happens later in reconcile(), from the complete, quiescent state.
 * The host rewrites this entry several times as the file grows and updates the FAT,
 * so acting now would act on partial state (an intermediate size, an unstaged or
 * not-yet-chained cluster); pending_add keeps the latest size, and reconcile flushes
 * once everything is present. size 0 is an in-progress entry (a real empty file has
 * first cluster 0 and never reaches here) - skip it, the true size comes later. */
static void try_flush(const char *vpath, uint32_t start, uint32_t size)
{
    if (size == 0) return;
    pending_add(vpath, start, size);
}

/* Resolve directory slot `g` (32-byte entry index) in directory `dir` to the model
 * item whose LFN+short chain covers it: full vpath into `out`, first cluster into
 * `*clus`. False for a lead slot (vol label / "." "..") or past the last item.
 * The inverse of emit_entry's layout, evaluated against the live model. */
static bool slot_resolve(int dir, uint32_t g, char *out, uint32_t outlen, uint32_t *clus)
{
    uint32_t lead = (dir < 0) ? 1 : 2;
    if (g < lead) return false;
    uint32_t pos = lead;
    /* Bound the scan so a corrupted dir/g can't spin forever; no real directory
     * approaches this many entries. */
    for (int n = 0; n < 65536; n++) {
        uint8_t attr; uint32_t start, size; int idx;
        const char *name = dir_item(dir, n, &attr, &start, &size, &idx);
        if (!name) return false;
        uint32_t k = name_entries(name);
        if (g < pos + k) {
            const char *prefix = (dir < 0) ? "" : s_dir[dir].vpath;
            if ((size_t)snprintf(out, outlen, "%s/%s", prefix, name) >= outlen)
                return false;                 /* path too long to reconstruct */
            *clus = start;
            return true;
        }
        pos += k;
    }
    return false;
}

/* Decode a directory sector's worth of entries. LFN entries (attr 0x0F) are
 * accumulated into `lfn`; the following short entry uses the assembled long name
 * (or its own 8.3 name if none). */
static void write_dir(int dir, const char *prefix, uint32_t slot0,
                      uint32_t offset, const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < EPS; i++) {
        uint32_t eoff = i * 32;
        if (eoff < offset || eoff + 32 > offset + len) continue;
        const uint8_t *e = buf + (eoff - offset);
        uint8_t c0 = e[0], attr = e[11];

        if (c0 == 0x00 || c0 == 0xE5) {       /* free / deleted directory slot */
            /* A 0xE5 marks the name unused, but that is ambiguous (delete vs. the
             * old half of a rename). Tombstone it (deferred): a later new entry at
             * the same cluster claims it as a rename; otherwise it commits as a real
             * delete on the next directory read (tomb_flush). */
            if (attr != 0x0F && c0 == 0xE5) {
                char vp[VPATH_MAX]; uint32_t clus;
                if (slot_resolve(dir, slot0 + i, vp, sizeof vp, &clus))
                    tomb_add(vp, clus);
            }
            continue;
        }
        if (attr == 0x0F) {                   /* LFN fragment: accumulate by checksum */
            int seq = (e[0] & 0x1f) - 1;
            int sl  = lfn_slot(e[13]);        /* e[13] = the short entry's checksum */
            static const int pos[13] = {1,3,5,7,9, 14,16,18,20,22,24, 28,30};
            if (sl >= 0 && seq >= 0 && seq < 20) {
                for (int j = 0; j < LFN_CHARS; j++) {
                    uint16_t ch = e[pos[j]] | (e[pos[j]+1] << 8);
                    int ci = seq * LFN_CHARS + j;
                    if (ch == 0x0000 || ch == 0xFFFF) { if (ci < (int)sizeof(s_lfnp[sl].name)) s_lfnp[sl].name[ci] = '\0'; }
                    else if (ci < (int)sizeof(s_lfnp[sl].name) - 1) s_lfnp[sl].name[ci] = (char)(ch & 0xff);
                }
            }
            continue;
        }
        if (attr & 0x08) continue;            /* volume label */
        if (e[0] == '.') continue;            /* "." ".." */

        char nm[64];
        int sl = lfn_find(sfn_cksum(e));      /* the fragments that named this entry */
        /* Keep the pool entry (don't release): the host rewrites a directory cluster
         * as it appends files, so write_dir re-processes a short entry whose LFN
         * fragments live in another cluster that isn't part of this write. Holding
         * the reconstructed name (until the mount reset clears the pool) keeps the
         * re-processed entry from collapsing to its 8.3 form. */
        if (sl >= 0 && s_lfnp[sl].name[0]) { strncpy(nm, s_lfnp[sl].name, sizeof(nm)-1); nm[sizeof(nm)-1]='\0'; }
        else {                                /* no long name: fall back to the 8.3 name */
            int o = 0;
            for (int k = 0; k < 8 && e[k] != ' '; k++) { char ch=e[k]; nm[o++]=(ch>='A'&&ch<='Z')?ch+32:ch; }
            if (e[8] != ' ') { nm[o++]='.'; for (int k=0;k<3&&e[8+k]!=' ';k++){char ch=e[8+k]; nm[o++]=(ch>='A'&&ch<='Z')?ch+32:ch;} }
            nm[o] = '\0';
        }
        if (nm[0] == '\0') continue;

        char vpath[VPATH_MAX];
        snprintf(vpath, VPATH_MAX, "%s/%s", prefix, nm);

        if (attr & 0x10) {                    /* subdirectory: create it */
            vfs_mkdir(vpath);
            uint32_t dclus = e[26] | (e[27] << 8) |   /* FAT32 full first cluster */
                             (e[20] << 16) | ((uint32_t)e[21] << 24);
            if (dclus >= 2 && !newdir_lookup(dclus) && s_nnewdir < MAX_NEWDIR) {
                s_newdir[s_nnewdir].cluster = dclus;
                strncpy(s_newdir[s_nnewdir].vpath, vpath, VPATH_MAX-1);
                s_newdir[s_nnewdir].vpath[VPATH_MAX-1] = '\0';
                s_nnewdir++;
            }
            continue;
        }

        uint32_t start = e[26] | (e[27] << 8) |
                         (e[20] << 16) | ((uint32_t)e[21] << 24);
        uint32_t size  = e[28] | (e[29]<<8) | (e[30]<<16) | ((uint32_t)e[31]<<24);
        if (start < 2) continue;

        /* A new entry with no staged data that reuses an existing first cluster is a
         * metadata-only rename/move: the file's data didn't move, only its name.
         * The source is either a tombstoned (just-0xE5'd) name or a still-live one.
         * Rename in place; copy across backends. Otherwise it's a real new file. */
        uint32_t nclus = size ? (size + BPS - 1) / BPS : 1;
        const char *from = NULL;
        bool from_tomb = false;
        char tvp[VPATH_MAX];
        if (!stage_find(start)) {
            synfile_t *live;
            if (tomb_claim(start, tvp))          { from = tvp; from_tomb = true; }  /* renamed-from a 0xE5 */
            else if ((live = file_of_cluster(start))) from = live->vpath;
        }
        if (from && strcmp(from, vpath) != 0) {
            if (vfs_rename(from, vpath) == 0) {
                asn_rename(from, vpath);          /* destination keeps the source's cluster */
            } else {                              /* no backend rename (cross-FS / ramfs): copy */
                const uint8_t *data; uint32_t dlen; bool owned;
                if (vfs_read_all(from, &data, &dlen, &owned) == 0) {
                    vfs_write_file(vpath, data, dlen);
                    asn_put(vpath, start, nclus);
                    if (owned) vPortFree((void *)data);
                    /* Complete the move: the host 0xE5'd the source, so drop it
                     * (a live-source match is a copy, not a move - leave it). */
                    if (from_tomb) vfs_remove(from);
                }
            }
        } else if (!from) {
            if (cluster_is_passthrough(start)) {
                /* Internal-subtree file whose data the host put in the card scratch
                 * (the internal region offers no free clusters). The host writes a
                 * file's data + FAT chain before this final directory entry, so the
                 * data is complete now: read it back off the card and commit it to
                 * the VFS. (An empty file - size 0 - just creates the entry.) */
                uint8_t *b = pvPortMalloc(size ? size : 1);
                if (b) {
                    if (size == 0 || ext_read_file(start, size, b) == 0) {
                        vfs_write_file(vpath, b, size);
                        if (size) ext_free_chain(start);   /* reclaim the card scratch */
                    }
                    vPortFree(b);
                }
            } else {
                try_flush(vpath, start, size);
                asn_put(vpath, start, nclus);     /* pin the file to the cluster the host chose */
            }
        }
    }
}

/* Capture one 512-byte sector's worth of host write (`len` bytes at `offset`). */
static int write_one_sector(uint32_t lba, uint32_t offset, const uint8_t *buf, uint32_t len)
{
    /* Reserved region (boot, FSInfo, backup): synthesized, ignore host writes. */
    if (lba < g_fat_lba) return 0;

    /* FAT region: the card's own FAT sectors are passed straight through (the host
     * allocates/frees clusters on the card's real filesystem); the internal FAT is
     * synthesized, so host writes to it are dropped (deletes arrive as directory
     * 0xE5 slots - see write_dir; a stale FAT write could free the wrong file). */
    if (lba < g_fat_lba + NFAT * g_fatsz) {
        /* Any FAT sector holding a card cluster is passed through so the host's
         * cluster (de)allocation lands on the card's real FAT; a pure-internal FAT
         * sector is synthetic, so its writes are dropped. */
        uint32_t fsec  = (lba - g_fat_lba) % g_fatsz;
        uint32_t first = fsec * 128u;
        if (g_pt_max && first <= g_pt_max) return ext_write_sector(g_pt_fat_lba + fsec, offset, buf, len);
        /* Internal FAT sector: synthetic, so we don't persist the host's allocations,
         * but two things in it we DO learn: (1) the cluster chains, so a fragmented
         * file reassembles from its real order (chain_record); (2) a link that chains
         * a fresh cluster onto a directory's tail, so entry writes there route to that
         * directory (s_dirext). */
        if (offset == 0 && len == BPS) {
            build_model();
            if (s_file) for (uint32_t i = 0; i < 128u; i++) {
                uint32_t A = first + i;
                if (A < 2) continue;
                uint32_t B = ((uint32_t)buf[i*4] | ((uint32_t)buf[i*4+1] << 8) |
                              ((uint32_t)buf[i*4+2] << 16) | ((uint32_t)buf[i*4+3] << 24)) & 0x0FFFFFFFu;
                if (B < 2 || B >= 0x0FFFFFF8u) continue;   /* not a forward chain link */
                if (B != A + 1) chain_record(A, B);        /* remember the non-contiguous jump */
                int own = dir_owner(A);                    /* A must already be directory space */
                if (own == -2 || dir_owner(B) != -2) continue;   /* B already known/owned */
                if (s_ndirext < MAX_DIREXT) {
                    s_dirext[s_ndirext].cluster = B; s_dirext[s_ndirext].dir = (int16_t)own; s_ndirext++;
                }
            }
        }
        return 0;
    }

    build_model();
    if (!s_file) return 0;   /* model couldn't allocate (OOM): drop the write rather than deref NULL */

    uint32_t rel   = lba - g_data_lba;
    uint32_t clus  = rel / g_sec_per_clus + 2u;
    uint32_t intra = rel % g_sec_per_clus;

    /* Card window: dir/data/FAT are the card's real structures - pass writes
     * straight to the card so the host maintains its filesystem for us. */
    if (cluster_is_passthrough(clus))
        return ext_write_sector(g_pt_data_lba + (clus - 2u) * g_sec_per_clus + intra, offset, buf, len);

    if (cluster_is_root(clus)) {
        uint32_t slot0 = ((clus - s_root_start) * g_sec_per_clus + intra) * EPS;
        write_dir(-1, "", slot0, offset, buf, len);
        return 0;
    }
    int d = dir_of_cluster(clus);
    if (d >= 0) {
        uint32_t slot0 = ((clus - s_dir[d].start) * g_sec_per_clus + intra) * EPS;
        write_dir(d, s_dir[d].vpath, slot0, offset, buf, len);
        return 0;
    }
    /* Write into a subdirectory the host just created this mount (its cluster is
     * not yet the one our model assigns). Route it to that directory. */
    const char *nd = newdir_lookup(clus);
    if (nd) {
        int ndi = dir_index(nd);
        if (ndi >= 0) { write_dir(ndi, nd, 0, offset, buf, len); return 0; }
    }
    /* A cluster the host chained onto a directory to hold more entries (learned from
     * its FAT write above): route its entry writes to that directory. */
    for (int i = 0; i < s_ndirext; i++) {
        if (s_dirext[i].cluster == clus) {
            int own = s_dirext[i].dir;
            if (own == -1) write_dir(-1, "", 0, offset, buf, len);
            else if (own >= 0 && own < s_ndir) write_dir(own, s_dir[own].vpath, 0, offset, buf, len);
            return 0;
        }
    }
    if (offset != 0 || len < BPS) return 0;
    stage_t *st = stage_find(clus);
    if (!st) {
        /* Staging keeps the whole in-flight file in RAM, so a big host copy could exhaust
         * the heap. Refuse new sectors once free heap nears the reserve, so the host gets a
         * clean write error instead of running the device out of memory. */
        if (xPortGetFreeHeapSize() < STAGE_MIN_FREE + sizeof(stage_t)) return -1;
        st = pvPortMalloc(sizeof(stage_t)); if (!st) return -1;
        st->cluster = clus; st->next = s_stage; s_stage = st;
    }
    memcpy(st->data, buf, BPS);
    return 0;
}

/* Commit any file whose data + directory entry are both complete, without waiting
 * for the next read. The MSC layer calls this on the host's SCSI SYNCHRONIZE CACHE
 * (sent on sync/unmount), so files copied over the mounted drive become durable in
 * LittleFS even if the host never reads the drive again. Staging is kept (idempotent
 * positional writes); it is freed on the next fresh mount. */
void fatrd_sync(void)
{
    fatrd_store_lock();
    flush_pending();
    fatrd_store_unlock();
}

int fatrd_write(uint32_t lba, uint32_t offset, const uint8_t *buf, uint32_t len)
{
    if (!buf) return -1;
    /* Serialise against the proto/VFS writers and any concurrent MSC read: the
     * internal-subtree capture reconciles through vfs_write_file (LittleFS). No USB
     * emit happens here, so holding the lock across the whole capture is safe. */
    fatrd_store_lock();
    /* Bring the write-side reconcile buffers resident for this session (freed on eject).
     * If the heap can't spare them, fail the write cleanly rather than half-capture. */
    if (!recon_bufs_ensure()) { fatrd_store_unlock(); return -1; }
    int rc = 0;
    /* The transfer may span several sectors (CFG_TUD_MSC_EP_BUFSIZE > 512); split it
     * into per-sector captures. `offset` is nonzero only when the EP buffer is
     * smaller than a sector, in which case there is a single partial sector. */
    uint32_t off = offset, cur = lba, remaining = len;
    while (remaining) {
        if (cur >= g_total_sec || off >= BPS) { rc = -1; break; }
        uint32_t n = BPS - off; if (n > remaining) n = remaining;
        if (write_one_sector(cur, off, buf, n) < 0) { rc = -1; break; }
        buf += n; remaining -= n; off = 0; cur++;
    }
    fatrd_store_unlock();
    return rc;
}
