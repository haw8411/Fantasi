#include "fat_ramdisk.h"
#include "../../core/vfs.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

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

/* Every synthetic directory starts a fresh mount with this many spare clusters
 * (16 free 32-byte slots each). Its allocation then stays fixed for that mount:
 * repeatedly adding slack after each new entry would grow the directory into a
 * cluster the mounted host had already allocated to file data. If the initial
 * slack fills, FAT-link capture below records the host's real extension chain. */
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
static __attribute__((noinline)) int ext_read_file(uint32_t start, uint32_t size, uint8_t *dst)
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
static __attribute__((noinline)) void ext_free_chain(uint32_t start)
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
typedef struct {
    char vpath[VPATH_MAX];
    uint32_t start, nclus, sfn_slot;
    uint8_t sfn[11];
    uint8_t flags;
} asn_t;
#define ASN_SFN_VALID       0x01u
#define ASN_SNAPSHOT_SEEN   0x02u
#define ASN_TOMB_AMBIGUOUS  0x04u
#define ASN_SFN_CASE_MASK   0x18u            /* FAT NTRes lowercase base/ext bits */
static asn_t    *s_asn;
static int       s_nasn;
static uint32_t  s_hiwater = 2;           /* next never-yet-used cluster this mount */
#define DATA_CLUSTERS (g_clusters)   /* usable cluster count (runtime) */

/* A directory created by this mounted host initially owns exactly the cluster
 * Linux selected for it. Keep that layout until a fresh mount; immediately
 * adding our normal synthetic-directory slack would make the still-mounted host
 * allocate the adjacent cluster to a file while firmware treats it as directory
 * space. The array is already part of the lazy MSC model allocation. */
#define MAX_NEWDIR 8
static struct { uint32_t cluster; bool reconciled; char vpath[VPATH_MAX]; } *s_newdir;
static int s_nnewdir;

/* Host-allocated clusters chained onto a full synthetic directory. The mapping
 * is part of the existing lazy reconciliation allocation and survives model
 * rebuilds for the duration of one mounted-host view. */
#define MAX_DIREXT 12
static struct dirext_s { uint32_t cluster; int16_t dir; } *s_dirext;
static int s_ndirext;

static uint32_t newdir_cluster_for_path(const char *vpath)
{
    for (int i = 0; i < s_nnewdir; i++)
        if (strcmp(s_newdir[i].vpath, vpath) == 0)
            return s_newdir[i].cluster;
    return 0;
}

static bool vpath_at_or_below(const char *candidate, const char *base)
{
    size_t n = strlen(base);
    return strncmp(candidate, base, n) == 0 &&
           (candidate[n] == '\0' || candidate[n] == '/');
}

static void newdir_forget(const char *vpath)
{
    for (int i = 0; i < s_nnewdir; ) {
        if (vpath_at_or_below(s_newdir[i].vpath, vpath)) {
            memmove(s_newdir + i, s_newdir + i + 1,
                    (s_nnewdir - i - 1) * sizeof s_newdir[0]);
            s_nnewdir--;
        } else {
            i++;
        }
    }
}

static void newdir_rename(const char *from, const char *to)
{
    size_t fn = strlen(from), tn = strlen(to);
    for (int i = 0; i < s_nnewdir; i++) {
        if (!vpath_at_or_below(s_newdir[i].vpath, from)) continue;
        const char *tail = s_newdir[i].vpath + fn;
        size_t tailn = strlen(tail);
        if (tn + tailn >= VPATH_MAX) {
            memmove(s_newdir + i, s_newdir + i + 1,
                    (s_nnewdir - i - 1) * sizeof s_newdir[0]);
            s_nnewdir--;
            i--;
            continue;
        }
        char rebased[VPATH_MAX];
        memcpy(rebased, to, tn);
        memcpy(rebased + tn, tail, tailn + 1);
        memcpy(s_newdir[i].vpath, rebased, tn + tailn + 1);
    }
}

static const char *leaf(const char *vpath)
{ const char *s = strrchr(vpath, '/'); return s ? s + 1 : vpath; }

/* This code runs in TinyUSB's deliberately small task. Pulling picolibc's
 * printf formatter into the directory read/write paths costs hundreds of bytes
 * of transient stack for operations that are only bounded copies and joins. */
static bool vpath_copy(char *out, size_t cap, const char *src)
{
    size_t n = strnlen(src, cap);
    if (n >= cap) { if (cap) out[0] = '\0'; return false; }
    memcpy(out, src, n + 1);
    return true;
}

static bool vpath_join(char *out, size_t cap, const char *prefix, const char *name)
{
    size_t a = strnlen(prefix, cap);
    if (a >= cap || a + 1 >= cap) { if (cap) out[0] = '\0'; return false; }
    size_t room = cap - a - 1;             /* reserve the slash */
    size_t b = strnlen(name, room);
    if (b >= room) { out[0] = '\0'; return false; }
    memcpy(out, prefix, a);
    out[a] = '/';
    memcpy(out + a + 1, name, b + 1);
    return true;
}

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
        if (!vpath_join(s_dir[s_ndir].vpath, VPATH_MAX, "", name)) return;
        s_dir[s_ndir].parent = -1;
        s_dir[s_ndir].is_ext = false;
        s_ndir++;
    } else {
        if ((unsigned)s_nfile >= s_cap_file) { s_of_file = true; return; }
        if (!vpath_join(s_file[s_nfile].vpath, VPATH_MAX, "", name)) return;
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
        if (!vpath_join(s_dir[s_ndir].vpath, VPATH_MAX, c->base, name)) return;
        s_dir[s_ndir].parent = c->dir;
        s_dir[s_ndir].is_ext = false;
        s_ndir++;
    } else {
        if ((unsigned)s_nfile >= s_cap_file) { s_of_file = true; return; }
        if (!vpath_join(s_file[s_nfile].vpath, VPATH_MAX, c->base, name)) return;
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

/* The synthetic model is derived from the live filesystem. Rebuild it only
 * after a mutation because each traversal is O(files). fatrd_write() and
 * fatrd_invalidate() advance s_fs_gen; build_model() is otherwise a no-op. */
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

/* External VFS mutations leave a mounted host's cached directory stale, so they
 * raise both model generation and removable-media notification. MSC-originated
 * commits use fatrd_invalidate_msc(): the host already owns that update and its
 * cluster assignment is pinned in s_asn, so reporting UNIT ATTENTION back to the
 * writer would make Linux invalidate the mounted block device mid-upload. */
void fatrd_invalidate(void)
{
    s_fs_gen++;
    s_media_dirty = true;
    s_ext_gen++;
}

void fatrd_invalidate_msc(void)
{
    s_fs_gen++;
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
static void pending_drop(const char *vpath, uint32_t start); /* retire a truly deleted file's staged tail */
static bool flush_pending(void);                      /* deferred-reconcile sweep; defined in the write section */
static void stage_free_all(void);                     /* drop all staged sectors; defined in the write section */
static uint32_t asn_assign(const char *vpath, uint32_t nclus);   /* stable cluster; defined after model_ensure */
static void asn_put(const char *vpath, uint32_t start, uint32_t nclus);
static void write_dir(int dir, const char *prefix, uint32_t slot0,
                      uint32_t offset, const uint8_t *buf, uint32_t len);

static bool asn_lookup(const char *vpath, uint32_t *start, uint32_t *nclus)
{
    for (int i = 0; i < s_nasn; i++)
        if (strcmp(s_asn[i].vpath, vpath) == 0) {
            if (start) *start = s_asn[i].start;
            if (nclus) *nclus = s_asn[i].nclus;
            return true;
        }
    return false;
}

/* Mark an existing assignment as still live during a model rebuild. A second
 * pass compacts away unmarked paths before asn_put adds genuinely new ones;
 * this preserves each survivor's exact mounted-host SFN without another heap
 * allocation or an in-place ordering hazard. */
static void asn_snapshot_mark(const char *vpath, uint32_t start, uint32_t nclus)
{
    for (int i = 0; i < s_nasn; i++)
        if (strcmp(s_asn[i].vpath, vpath) == 0) {
            s_asn[i].start = start;
            s_asn[i].nclus = nclus;
            s_asn[i].flags |= ASN_SNAPSHOT_SEEN;
            return;
        }
}

static void asn_snapshot_prune(void)
{
    int out = 0;
    for (int i = 0; i < s_nasn; i++) {
        if (!(s_asn[i].flags & ASN_SNAPSHOT_SEEN)) continue;
        s_asn[i].flags &= (uint8_t)~ASN_SNAPSHOT_SEEN;
        if (out != i) s_asn[out] = s_asn[i];
        out++;
    }
    s_nasn = out;
}

/* Runtime geometry helpers.  The SD-backed merged volume commonly uses 32 KiB
 * clusters (64 sectors); treating every 512-byte sector as a cluster corrupts
 * replacement chains and makes the host eventually free entries past EOF. */
static uint32_t file_nclus(uint32_t size)
{
    uint32_t bpc = BPS * g_sec_per_clus;
    return size ? (size + bpc - 1u) / bpc : 1u;
}

static uint32_t dir_nclus(uint32_t entries)
{
    uint32_t epc = EPS * g_sec_per_clus;
    return (entries + epc - 1u) / epc;
}

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
    for (int i = 0; i < s_nnewdir; i++)
        if (s_newdir[i].cluster + 1 > s_hiwater)
            s_hiwater = s_newdir[i].cluster + 1;
    for (int i = 0; i < s_ndirext; i++)
        if (s_dirext[i].cluster + 1 > s_hiwater)
            s_hiwater = s_dirext[i].cluster + 1;

    /* Stable cluster layout: each item keeps the cluster it already holds; only
     * new/resized ones draw fresh blocks. The FAT32 root goes first so it lands on
     * the lowest internal cluster (g_int_first). Then snapshot the assignment so the
     * next rebuild (and rename detection) resolves the same clusters. */
    {
        uint32_t rents = dir_entry_count(-1);
        if (!asn_lookup("/", &s_root_start, &s_root_nclus)) {
            s_root_nclus = dir_nclus(rents) + DIR_SLACK_CLUS;
            s_root_start = asn_assign("/", s_root_nclus);
        }
    }
    for (int d = 0; d < s_ndir; d++) {
        if (s_dir[d].is_ext) {                 /* passthrough mount root -> card's root cluster */
            s_dir[d].start = g_ext_root_clus;
            s_dir[d].nclus = 0;                /* owns no synthetic clusters */
            continue;
        }
        uint32_t ents = dir_entry_count(d);
        uint32_t host_start = newdir_cluster_for_path(s_dir[d].vpath);
        if (asn_lookup(s_dir[d].vpath, &s_dir[d].start, &s_dir[d].nclus)) {
            /* Never resize or move a directory underneath this mounted host.
             * Entries beyond this base allocation use s_dirext's captured chain. */
        } else if (host_start) {
            s_dir[d].nclus = dir_nclus(ents);
            s_dir[d].start = host_start;
            uint32_t end = host_start + s_dir[d].nclus;
            if (end > s_hiwater) s_hiwater = end;
        } else {
            s_dir[d].nclus = dir_nclus(ents) + DIR_SLACK_CLUS;
            s_dir[d].start = asn_assign(s_dir[d].vpath, s_dir[d].nclus);
        }
    }
    for (int i = 0; i < s_nfile; i++) {
        s_file[i].nclus = file_nclus(s_file[i].size);
        s_file[i].start = asn_assign(s_file[i].vpath, s_file[i].nclus);
    }
    for (int i = 0; i < s_nasn; i++)
        s_asn[i].flags &= (uint8_t)~ASN_SNAPSHOT_SEEN;
    asn_snapshot_mark("/", s_root_start, s_root_nclus);
    for (int d = 0; d < s_ndir; d++)
        asn_snapshot_mark(s_dir[d].vpath, s_dir[d].start, s_dir[d].nclus);
    for (int i = 0; i < s_nfile; i++)
        asn_snapshot_mark(s_file[i].vpath, s_file[i].start, s_file[i].nclus);
    asn_snapshot_prune();

    asn_put("/", s_root_start, s_root_nclus);
    for (int d = 0; d < s_ndir; d++)
        asn_put(s_dir[d].vpath, s_dir[d].start, s_dir[d].nclus);
    for (int i = 0; i < s_nfile; i++)
        asn_put(s_file[i].vpath, s_file[i].start, s_file[i].nclus);
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

/* During a cross-backend FAT move the new directory entry can be decoded before
 * the old one is tombstoned, so two transient model paths legitimately name the
 * same cluster. Search all matches for the directory being decoded instead of
 * rejecting whichever global match happened to appear first. */
static synfile_t *file_of_cluster_parent(uint32_t c, int parent)
{
    for (int i = 0; i < s_nfile; i++)
        if (s_file[i].parent == parent &&
            c >= s_file[i].start && c < s_file[i].start + s_file[i].nclus)
            return &s_file[i];
    return NULL;
}

static synfile_t *file_of_path(const char *vpath)
{
    for (int i = 0; i < s_nfile; i++)
        if (strcmp(s_file[i].vpath, vpath) == 0) return &s_file[i];
    return NULL;
}

/* When the host creates a subdirectory mid-mount it allocates a cluster for it
 * out of the FAT free list - a cluster our model will assign elsewhere on the
 * next rebuild. We remember that host-chosen cluster here so entry writes the
 * host makes into the new directory (e.g. a file created inside it before the
 * next remount) are routed to the right directory instead of being mistaken for
 * raw file data. Cleared on every mount (boot-sector read) so a stale mapping
 * can't misroute a later session's writes. */
/* Host-allocated clusters that extend a directory past the model's contiguous
 * range: when a directory's presented clusters fill up, the host chains a fresh
 * free cluster onto the tail and writes further entries there. The synthetic FAT
 * doesn't store the host's links, but we learn this one from its FAT write and
 * remember cluster -> owning directory, so the entry writes that follow route to
 * write_dir instead of being captured as raw file data (and silently lost).
 * dir: -1 = root, >=0 = s_dir index. Cleared each mount (boot-sector read). */
/* Deferred reconcile: a directory entry can arrive before all file data is
 * staged. `committed` lets sync retire completed prefixes. A zero start marks
 * an unused slot because FAT data begins at cluster 2. */
#define MAX_PENDING 16
static struct pending_s { char vpath[VPATH_MAX]; uint32_t start, size, committed; } *s_pending;   /* lazy heap */
/* If capture cannot retain a sector, drain the current BOT data phase and
 * reject the following SYNCHRONIZE CACHE at its durability boundary. */
static bool s_capture_failed;

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

static bool dirext_has(uint32_t cluster, int owner)
{
    for (int i = 0; i < s_ndirext; i++)
        if (s_dirext[i].cluster == cluster && s_dirext[i].dir == owner)
            return true;
    return false;
}

/* Return the captured successor of a directory cluster, if that successor is
 * itself one of this directory's host-allocated extensions. chain_next() also
 * covers the common contiguous-link case without spending a fatchain slot. */
static bool dirext_next(uint32_t cluster, int owner, uint32_t *next)
{
    uint32_t n = chain_next(cluster);
    if (!dirext_has(n, owner)) return false;
    if (next) *next = n;
    return true;
}

/* Locate an extension cluster in its owning directory's logical chain. `pos` is
 * its zero-based cluster position after the fixed base allocation, so directory
 * entry decoding/synthesis uses the same slots Linux wrote. */
static bool dirext_position(uint32_t cluster, int *owner_out, uint32_t *pos_out)
{
    int owner = -2;
    for (int i = 0; i < s_ndirext; i++)
        if (s_dirext[i].cluster == cluster) { owner = s_dirext[i].dir; break; }
    if (owner == -2) return false;

    uint32_t start, base;
    if (owner == -1) {
        start = s_root_start; base = s_root_nclus;
    } else if (owner >= 0 && owner < s_ndir) {
        start = s_dir[owner].start; base = s_dir[owner].nclus;
    } else {
        return false;
    }
    if (!base) return false;

    uint32_t cur = start + base - 1;
    for (uint32_t pos = base, steps = 0; steps < MAX_DIREXT; steps++, pos++) {
        uint32_t next;
        if (!dirext_next(cur, owner, &next)) return false;
        if (next == cluster) {
            if (owner_out) *owner_out = owner;
            if (pos_out) *pos_out = pos;
            return true;
        }
        cur = next;
    }
    return false;
}

/* Retire routes owned by a removed directory so repeated create/fill/delete
 * cycles do not exhaust the bounded extension table within one mount. */
static void dirext_forget(const char *vpath)
{
    for (int i = 0; i < s_ndirext; ) {
        int dir = s_dirext[i].dir;
        const char *owner = (dir >= 0 && dir < s_ndir) ? s_dir[dir].vpath : NULL;
        if (owner && vpath_at_or_below(owner, vpath)) {
            memmove(s_dirext + i, s_dirext + i + 1,
                    (s_ndirext - i - 1) * sizeof s_dirext[0]);
            s_ndirext--;
        } else {
            i++;
        }
    }
}

/* Partial long-name reconstruction. Linux may dirty the sector containing an SFN
 * before the preceding sector containing the first LFN fragment. Committing that
 * SFN immediately permanently exposes the host's generated XXXXXX~1 alias. Keep a
 * transaction for the exact owning directory + target SFN slot + checksum instead;
 * an unresolved SFN is retained until SYNCHRONIZE CACHE, by which point the other
 * sector has arrived. Checksum alone is not an identity: collisions are common and
 * whole-directory rewrites can interleave several names with the same byte sum. */
#define LFN_POOL 16
#define LFN_ACTIVE 0x80u
#define LFN_HAVE_SHORT 0x40u
/* `state` uses its low five bits as the received-fragment mask. The implementation
 * supports a 63-character leaf (five LFN fragments), matching the path model's
 * existing limit. `short_entry` is populated only when the SFN outruns a fragment. */
static struct lfnp_s {
    uint32_t owner;
    uint32_t sfn_slot;
    uint32_t order;
    uint8_t cksum, state, expected;
    uint8_t short_entry[32];
    char name[64];
} *s_lfnp;                                             /* lazy heap */
static uint32_t s_lfn_next_order;

/* Root uses owner 0 (not a legal FAT cluster); other directories keep their
 * mounted-host cluster even if a model rebuild reorders the s_dir array. */
static uint32_t lfn_owner(int dir)
{
    if (dir < 0) return 0;
    return dir < s_ndir ? s_dir[dir].start : 0xFFFFFFFFu;
}

static int lfn_find(uint32_t owner, uint32_t sfn_slot, uint8_t ck)
{
    if (!s_lfnp) return -1;
    for (int i = 0; i < LFN_POOL; i++)
        if ((s_lfnp[i].state & LFN_ACTIVE) &&
            s_lfnp[i].owner == owner && s_lfnp[i].sfn_slot == sfn_slot &&
            s_lfnp[i].cksum == ck)
            return i;
    return -1;
}

static int lfn_slot(uint32_t owner, uint32_t sfn_slot, uint8_t ck)
{
    if (!s_lfnp) return -1;
    int i = lfn_find(owner, sfn_slot, ck);
    if (i >= 0) return i;
    /* Reuse of one physical directory slot supersedes its older transaction,
     * even when the new SFN has a different checksum. */
    for (i = 0; i < LFN_POOL; i++)
        if ((s_lfnp[i].state & LFN_ACTIVE) &&
            s_lfnp[i].owner == owner && s_lfnp[i].sfn_slot == sfn_slot)
            memset(&s_lfnp[i], 0, sizeof s_lfnp[i]);
    for (i = 0; i < LFN_POOL; i++) if (!(s_lfnp[i].state & LFN_ACTIVE)) {
        memset(&s_lfnp[i], 0, sizeof s_lfnp[i]);
        s_lfnp[i].state = LFN_ACTIVE;
        s_lfnp[i].owner = owner;
        s_lfnp[i].sfn_slot = sfn_slot;
        s_lfnp[i].cksum = ck;
        return i;
    }
    return -1;
}

static bool lfn_complete(int slot)
{
    if (!s_lfnp || slot < 0 || slot >= LFN_POOL) return false;
    uint8_t expected = s_lfnp[slot].expected;
    if (!expected || expected > 5) return false;
    uint8_t mask = (uint8_t)((1u << expected) - 1u);
    return (s_lfnp[slot].state & mask) == mask;
}
static void lfn_release(int slot)
{
    if (s_lfnp && slot >= 0 && slot < LFN_POOL) {
        memset(&s_lfnp[slot], 0, sizeof s_lfnp[slot]);
    }
}

static void lfn_cancel(uint32_t owner, uint32_t sfn_slot)
{
    if (!s_lfnp) return;
    for (int i = 0; i < LFN_POOL; i++)
        if ((s_lfnp[i].state & LFN_ACTIVE) &&
            s_lfnp[i].owner == owner && s_lfnp[i].sfn_slot == sfn_slot)
            lfn_release(i);
}

static void lfn_cancel_from(uint32_t owner, uint32_t sfn_slot)
{
    if (!s_lfnp) return;
    for (int i = 0; i < LFN_POOL; i++)
        if ((s_lfnp[i].state & LFN_ACTIVE) &&
            s_lfnp[i].owner == owner && s_lfnp[i].sfn_slot >= sfn_slot)
            lfn_release(i);
}

static void lfn_cancel_owner(uint32_t owner)
{
    if (!s_lfnp) return;
    for (int i = 0; i < LFN_POOL; i++)
        if ((s_lfnp[i].state & LFN_ACTIVE) && s_lfnp[i].owner == owner)
            lfn_release(i);
}

static bool lfn_has_active_transaction(void)
{
    if (!s_lfnp) return false;
    for (int i = 0; i < LFN_POOL; i++)
        if (s_lfnp[i].state & LFN_ACTIVE) return true;
    return false;
}

static bool lfn_has_unresolved_transaction(void)
{
    if (!s_lfnp) return false;
    for (int i = 0; i < LFN_POOL; i++)
        if ((s_lfnp[i].state & LFN_ACTIVE) &&
            (!(s_lfnp[i].state & LFN_HAVE_SHORT) || !lfn_complete(i)))
            return true;
    return false;
}

/* Replay retained SFNs only at a write durability/session boundary, never on an
 * intervening directory read. Complete transactions consume their reconstructed
 * LFN. A real SCSI sync may fall an incomplete transaction back to native 8.3;
 * an asynchronous USB unplug may not, because a missing fragment can mean the
 * write itself was interrupted. */
static bool s_lfn_force_short;
static bool s_lfn_ordered_recovery;
static void lfn_pending_flush(bool fallback_incomplete)
{
    if (!s_lfnp) return;
    for (;;) {
        int i = -1;
        uint32_t oldest = 0;
        for (int k = 0; k < LFN_POOL; k++) {
            if ((s_lfnp[k].state & (LFN_ACTIVE | LFN_HAVE_SHORT)) !=
                (LFN_ACTIVE | LFN_HAVE_SHORT))
                continue;
            if (!fallback_incomplete && !lfn_complete(k)) continue;
            if (i < 0 || s_lfnp[k].order < oldest) {
                i = k;
                oldest = s_lfnp[k].order;
            }
        }
        if (i < 0) break;

        /* A prior replay (or proto/BLE mutation) may have renamed the owner or
         * the file this SFN points at. Refresh before resolving stable clusters. */
        build_model();
        uint8_t entry[32];
        memcpy(entry, s_lfnp[i].short_entry, sizeof entry);
        uint32_t owner = s_lfnp[i].owner;
        uint32_t slot = s_lfnp[i].sfn_slot;
        int dir = -1;
        const char *prefix;
        if (!owner) prefix = "";
        else if ((dir = dir_of_cluster(owner)) >= 0 && dir < s_ndir)
            prefix = s_dir[dir].vpath;
        else {
            s_capture_failed = true;
            lfn_release(i);
            continue;
        }

        bool complete = lfn_complete(i);
        if (!complete) lfn_release(i);          /* make replay take the SFN path */
        s_lfn_force_short = !complete;
        write_dir(dir, prefix, slot, 0, entry, sizeof entry);
        s_lfn_force_short = false;
    }
    /* Fragments without a retained SFN belong to harmless whole-sector rewrites,
     * or to an abandoned transaction. Do not let either consume the bounded pool
     * for the rest of a long-lived composite-USB mount. */
    for (int i = 0; i < LFN_POOL; i++) lfn_release(i);
}

static uint8_t sfn_cksum(const uint8_t *e)
{ uint8_t s = 0; for (int i = 0; i < 11; i++) s = (uint8_t)(((s & 1) << 7) + (s >> 1) + e[i]); return s; }

/* Decode a short-only 8.3 entry. FAT stores letters uppercase and uses NTRes
 * bit 3/4 to request a lowercase base/extension; preserving those flags matters
 * for a real short-name rename such as a.txt -> b.txt. */
static void sfn_decode_name(const uint8_t *e, char out[13])
{
    bool lower_base = (e[12] & 0x08u) != 0;
    bool lower_ext  = (e[12] & 0x10u) != 0;
    int o = 0;
    for (int k = 0; k < 8 && e[k] != ' '; k++) {
        uint8_t ch = e[k];
        if (k == 0 && ch == 0x05) ch = 0xE5;  /* escaped leading 0xE5 */
        if (lower_base && ch >= 'A' && ch <= 'Z') ch += 'a' - 'A';
        out[o++] = (char)ch;
    }
    if (e[8] != ' ') {
        out[o++] = '.';
        for (int k = 8; k < 11 && e[k] != ' '; k++) {
            uint8_t ch = e[k];
            if (lower_ext && ch >= 'A' && ch <= 'Z') ch += 'a' - 'A';
            out[o++] = (char)ch;
        }
    }
    out[o] = '\0';
}

/* Encode an ASCII leaf that FAT can represent natively as a single 8.3 entry.
 * A component containing both upper- and lowercase letters needs an LFN, as do
 * spaces, extra dots, overlong fields and characters FAT aliases lossily. */
static bool sfn_encode_native(const char *name, uint8_t out[11], uint8_t *ntres)
{
    const char *dot = strchr(name, '.');
    size_t base_len = dot ? (size_t)(dot - name) : strlen(name);
    size_t ext_len = dot ? strlen(dot + 1) : 0;
    if (!base_len || base_len > 8 || ext_len > 3 ||
        (dot && (!ext_len || strchr(dot + 1, '.'))))
        return false;

    memset(out, ' ', 11);
    *ntres = 0;
    for (int field = 0; field < 2; field++) {
        const char *p = field ? (dot ? dot + 1 : "") : name;
        size_t len = field ? ext_len : base_len;
        size_t off = field ? 8u : 0u;
        bool lower = false, upper = false;
        for (size_t i = 0; i < len; i++) {
            unsigned char ch = (unsigned char)p[i];
            /* The remaining printable ASCII punctuation is legal in an SFN.
             * These characters are either invalid or force a lossy/LFN alias. */
            if (ch <= ' ' || ch >= 0x7Fu ||
                strchr("\"*+,/:;<=>?[\\]|", ch))
                return false;
            if (ch >= 'a' && ch <= 'z') {
                lower = true;
                ch = (unsigned char)(ch - ('a' - 'A'));
            } else if (ch >= 'A' && ch <= 'Z') {
                upper = true;
            }
            out[off + i] = ch;
        }
        if (lower && upper) return false;
        if (lower) *ntres |= field ? 0x10u : 0x08u;
    }
    return true;
}

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

    /* Free refillable buffers before allocating replacements to limit peak
     * heap. Preserve only the persistent cluster assignments. */
    vPortFree(s_dir);  s_dir  = NULL;
    vPortFree(s_file); s_file = NULL;
    s_cap_dir = s_cap_file = 0;

    asn_t *nasn = pvPortMalloc((nd + nf + 1u) * sizeof *nasn); /* + root assignment */
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
 * the pending/LFN scans need a guard. */
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
    memset(s_pending, 0, MAX_PENDING * sizeof *s_pending);   /* start = unused */
    memset(s_lfnp,    0, LFN_POOL    * sizeof *s_lfnp);      /* .state = inactive */
    s_lfn_next_order = 0;
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

/* Internal VFS objects created through the merged volume temporarily borrow
 * ordinary SD clusters.  Keep those chains intact while the host has that FAT
 * view mounted, then release each unique chain at the next mount/eject boundary.
 * The passthrough /mnt/extN roots have nclus == 0 and are never scratch. */
static void ext_scratch_free_all(void)
{
    for (int i = 0; i < s_nasn; i++) {
        uint32_t start = s_asn[i].start;
        if (!s_asn[i].nclus || !cluster_is_passthrough(start)) continue;
        bool seen = false;
        for (int j = 0; j < i; j++)
            if (s_asn[j].nclus && s_asn[j].start == start) { seen = true; break; }
        if (!seen) ext_free_chain(start);
    }
}

void fatrd_release(void)
{
    fatrd_store_lock();
    /* Eject normally follows SYNCHRONIZE CACHE, but unplug paths can arrive
     * without it. Resolve a retained SFN before its source tombstone, then make
     * any staged data durable before freeing the reconstruction state. */
    if (!s_capture_failed) {
        bool interrupted_name = lfn_has_unresolved_transaction();
        lfn_pending_flush(false);
        if (!s_capture_failed && !interrupted_name) {
            /* An incomplete rename destination may own one of these source
             * tombstones. On abrupt unplug, preserving an extra source is safer
             * than deleting it without its destination; without a durability
             * boundary its other staged writes are ambiguous too. */
            tomb_flush();
            (void)flush_pending();
        }
    }
    ext_scratch_free_all();
    vPortFree(s_dir); vPortFree(s_file); vPortFree(s_newdir); vPortFree(s_asn);
    s_dir = NULL; s_file = NULL; s_newdir = NULL; s_asn = NULL;
    s_cap_dir = s_cap_file = 0; s_nasn = 0; s_hiwater = 2;
    s_have_model = false; s_nnewdir = 0;                 /* next access rebuilds from scratch */
    stage_free_all();
    recon_bufs_free();                                   /* hand the ~5 KB back to the idle-time heap */
    s_capture_failed = false;
    fatrd_store_unlock();
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
    if ((unsigned)s_nasn < s_cap_dir + s_cap_file + 1u) {
        memset(&s_asn[s_nasn], 0, sizeof s_asn[s_nasn]);
        strncpy(s_asn[s_nasn].vpath, vpath, VPATH_MAX - 1);
        s_asn[s_nasn].vpath[VPATH_MAX - 1] = '\0';
        s_asn[s_nasn].start = start; s_asn[s_nasn].nclus = nclus; s_nasn++;
    }
}

/* Remember the exact raw SFN the mounted host currently associates with this
 * stable path/cluster. It may be our FAPxxxxx alias or Linux's own alias for a
 * host-created long name; retaining the bytes makes later partial-sector
 * rewrites unambiguous even after model indices move. */
static void asn_sfn_record(const char *vpath, uint32_t start,
                           const uint8_t *sfn, uint8_t ntres, uint32_t slot)
{
    for (int i = 0; i < s_nasn; i++)
        if (s_asn[i].start == start && strcmp(s_asn[i].vpath, vpath) == 0) {
            /* Deletion destroys SFN byte 0. If a rename changes only that byte
             * in the same slot (A.TXT -> B.TXT), a future replay of the source
             * tombstone is indistinguishable from deletion of the destination.
             * Reject such tombstones for the rest of this mount. */
            if ((s_asn[i].flags & ASN_SFN_VALID) &&
                s_asn[i].sfn_slot == slot && s_asn[i].sfn[0] != sfn[0] &&
                memcmp(s_asn[i].sfn + 1, sfn + 1, 10) == 0 &&
                (s_asn[i].flags & ASN_SFN_CASE_MASK) ==
                (ntres & ASN_SFN_CASE_MASK))
                s_asn[i].flags |= ASN_TOMB_AMBIGUOUS;
            memcpy(s_asn[i].sfn, sfn, sizeof s_asn[i].sfn);
            s_asn[i].sfn_slot = slot;
            s_asn[i].flags = (uint8_t)((s_asn[i].flags & ~ASN_SFN_CASE_MASK) |
                                      ASN_SFN_VALID | (ntres & ASN_SFN_CASE_MASK));
            return;
        }
}

static bool asn_sfn_compare(const char *vpath, uint32_t start,
                            const uint8_t *sfn, bool *matches)
{
    for (int i = 0; i < s_nasn; i++)
        if (s_asn[i].start == start && strcmp(s_asn[i].vpath, vpath) == 0 &&
            (s_asn[i].flags & ASN_SFN_VALID)) {
            *matches = memcmp(s_asn[i].sfn, sfn, sizeof s_asn[i].sfn) == 0 &&
                       (s_asn[i].flags & ASN_SFN_CASE_MASK) ==
                       (sfn[12] & ASN_SFN_CASE_MASK);
            return true;
        }
    return false;
}

static const char *asn_file_for_sfn_slot(int parent, uint32_t slot,
                                         const uint8_t *sfn)
{
    for (int i = 0; i < s_nasn; i++) {
        if (!(s_asn[i].flags & ASN_SFN_VALID) ||
            s_asn[i].sfn_slot != slot ||
            memcmp(s_asn[i].sfn, sfn, sizeof s_asn[i].sfn) != 0 ||
            (s_asn[i].flags & ASN_SFN_CASE_MASK) !=
            (sfn[12] & ASN_SFN_CASE_MASK))
            continue;
        synfile_t *f = file_of_path(s_asn[i].vpath);
        if (f && f->parent == parent) return s_asn[i].vpath;
    }
    return NULL;
}

/* A deleted FAT entry retains bytes 1..10 of its old SFN but overwrites byte 0
 * with 0xE5.  When the host replays a stale deleted slot after directory layout
 * changes, its cluster can now belong to a live path.  Use the retained suffix
 * and case flags to distinguish that replay from deletion of the alias this
 * mount actually exposed. Returns false when no exact identity was observed. */
static bool asn_deleted_sfn_compare(const char *vpath, uint32_t start,
                                    uint32_t slot, const uint8_t *entry,
                                    bool *matches)
{
    for (int i = 0; i < s_nasn; i++)
        if (s_asn[i].start == start && strcmp(s_asn[i].vpath, vpath) == 0 &&
            (s_asn[i].flags & ASN_SFN_VALID)) {
            *matches = !(s_asn[i].flags & ASN_TOMB_AMBIGUOUS) &&
                       s_asn[i].sfn_slot == slot &&
                       memcmp(s_asn[i].sfn + 1, entry + 1, 10) == 0 &&
                       (s_asn[i].flags & ASN_SFN_CASE_MASK) ==
                       (entry[12] & ASN_SFN_CASE_MASK);
            return true;
        }
    return false;
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

static void asn_forget(const char *path)
{
    for (int i = 0; i < s_nasn; i++)
        if (strcmp(s_asn[i].vpath, path) == 0) {
            memmove(s_asn + i, s_asn + i + 1,
                    (s_nasn - i - 1) * sizeof s_asn[0]);
            s_nasn--;
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
    uint32_t n = (uint32_t)idx % 100000u;
    out[0] = 'F'; out[1] = 'A'; out[2] = 'P';
    for (int i = 7; i >= 3; i--) { out[i] = (uint8_t)('0' + n % 10u); n /= 10u; }
    const char *dot = strrchr(name, '.');
    if (dot) for (int i = 0; i < 3 && dot[1+i]; i++) {
        char ch = dot[1+i];
        out[8+i] = (ch>='a'&&ch<='z') ? ch-32 : ch;
    }
}

static void asn_sfn_record_item(uint32_t start, const char *name,
                                const uint8_t *sfn, uint8_t ntres, uint32_t slot)
{
    for (int i = 0; i < s_nasn; i++)
        if (s_asn[i].start == start && strcmp(leaf(s_asn[i].vpath), name) == 0) {
            if ((s_asn[i].flags & ASN_SFN_VALID) &&
                s_asn[i].sfn_slot == slot && s_asn[i].sfn[0] != sfn[0] &&
                memcmp(s_asn[i].sfn + 1, sfn + 1, 10) == 0 &&
                (s_asn[i].flags & ASN_SFN_CASE_MASK) ==
                (ntres & ASN_SFN_CASE_MASK))
                s_asn[i].flags |= ASN_TOMB_AMBIGUOUS;
            memcpy(s_asn[i].sfn, sfn, sizeof s_asn[i].sfn);
            s_asn[i].sfn_slot = slot;
            s_asn[i].flags = (uint8_t)((s_asn[i].flags & ~ASN_SFN_CASE_MASK) |
                                      ASN_SFN_VALID | (ntres & ASN_SFN_CASE_MASK));
            return;
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
    if (cluster_is_root(e)) {
        if (e != s_root_start + s_root_nclus - 1) return e + 1;
        uint32_t next;
        return dirext_next(e, -1, &next) ? next : 0x0FFFFFFF;
    }
    for (int d = 0; d < s_ndir; d++)
        if (e >= s_dir[d].start && e < s_dir[d].start + s_dir[d].nclus) {
            if (e != s_dir[d].start + s_dir[d].nclus - 1) return e + 1;
            uint32_t next;
            return dirext_next(e, d, &next) ? next : 0x0FFFFFFF;
        }
    int owner; uint32_t next;
    if (dirext_position(e, &owner, NULL))
        return dirext_next(e, owner, &next) ? next : 0x0FFFFFFF;
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
                asn_sfn_record_item(start, name, sn, 0, g);
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
        pending_drop(s_tomb[0].vpath, s_tomb[0].cluster);
        if (vfs_msc_remove(s_tomb[0].vpath) == 0)
            newdir_forget(s_tomb[0].vpath);
        asn_forget(s_tomb[0].vpath);
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
    for (int i = 0; i < s_ntomb; i++) {
        pending_drop(s_tomb[i].vpath, s_tomb[i].cluster);
        if (vfs_msc_remove(s_tomb[i].vpath) == 0)
            newdir_forget(s_tomb[i].vpath);
        /* Its FAT chain was released by the mounted host. Drop the stale pin
         * before that cluster can be reused for another object. */
        asn_forget(s_tomb[i].vpath);
    }
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
        int d, owner;
        uint32_t pos;
        if (cluster_is_passthrough(clus)) {               /* card data/dir sector, raw */
            if (fatrd_ext_read(g_pt_data_lba + (clus - 2u) * g_sec_per_clus + intra, sec) != 0)
                memset(sec, 0, BPS);
        }
        else if (cluster_is_root(clus))
            synth_dir(-1, ((clus - s_root_start) * g_sec_per_clus + intra) * EPS, sec);
        else if ((d = dir_of_cluster(clus)) >= 0)
            synth_dir(d, ((clus - s_dir[d].start) * g_sec_per_clus + intra) * EPS, sec);
        else if (dirext_position(clus, &owner, &pos))
            synth_dir(owner, (pos * g_sec_per_clus + intra) * EPS, sec);
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

/* MSC normally asks for whole aligned sectors. Keep the 512-byte scratch needed
 * only by a partial read out of fatrd_read's normal stack frame: on the PM3 that
 * frame sits under TinyUSB's class/event call chain, and reserving it for every
 * aligned READ10 left barely 100 bytes of task-stack margin. */
static __attribute__((noinline)) int read_partial_sector(uint32_t lba,
                                                         uint32_t offset,
                                                         void *buf,
                                                         uint32_t len,
                                                         bool generation)
{
    uint8_t sec[BPS];
    if (generation) synth_gen(sec);
    else            fill_sector(lba, sec);
    if (offset >= BPS) return -1;
    if (offset + len > BPS) len = BPS - offset;
    memcpy(buf, sec + offset, len);
    return 0;
}

int fatrd_read(uint32_t lba, uint32_t offset, void *buf, uint32_t len)
{
    /* The generation sector is answered directly - no model rebuild - so the host's
     * pre-read freshness probe is a cheap single-sector read. */
    if (lba == GEN_LBA && offset == 0 && len == BPS) {
        synth_gen(buf);
        return 0;
    }
    if (lba == GEN_LBA)
        return read_partial_sector(lba, offset, buf, len, true);
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
    if (lba == 0) { if (!s_capture_failed) {
                        lfn_pending_flush(true);
                        if (!s_capture_failed) {
                            tomb_flush();
                            (void)flush_pending();
                        }
                    }
                    ext_scratch_free_all();
                    s_nnewdir = 0; s_ndirext = 0; s_nfatchain = 0; s_nasn = 0; s_hiwater = 2; s_have_model = false;
                    if (s_lfnp)    for (int k = 0; k < LFN_POOL; k++)   s_lfnp[k].state = 0;
                    if (s_pending) for (int k = 0; k < MAX_PENDING; k++) s_pending[k].start = 0;
                    stage_free_all();
                    s_capture_failed = false; }
    /* Any read may observe a directory listing, so commit deferred deletes first;
     * a rename writes its 0xE5 + new entry back-to-back with no read between, so
     * this never pre-empts an in-flight rename. */
    if (!lfn_has_active_transaction()) tomb_flush();
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

        if (off == 0 && remaining >= BPS) {
            uint32_t clba = passthrough_lba(cur);
            if (clba) {
                uint32_t k = 1, want = remaining / BPS;
                while (k < want && passthrough_lba(cur + k) == clba + k) k++;
                if (fatrd_ext_read_multi(clba, out, k) != 0) memset(out, 0, k * BPS);
                out += k * BPS; remaining -= k * BPS; cur += k;
            } else {
                fill_sector(cur, out);
                out += BPS; remaining -= BPS; cur++;
            }
        } else {
            uint32_t n = BPS - off; if (n > remaining) n = remaining;
            if (read_partial_sector(cur, off, out, n, false) < 0) {
                fatrd_store_unlock();
                return -1;
            }
            out += n; remaining -= n; off = 0; cur++;
        }
    }
    fatrd_store_unlock();
    return 0;
}

/* ---- WRITE (capture) ---- */

// Heap held in reserve while staging a host write
#define STAGE_MIN_FREE 6144u

/* One node is one 512-byte data sector, not one FAT cluster. SD-backed views
 * commonly have 64 sectors per cluster; cluster-only keys made every sector of
 * a replacement overwrite the same node. `sector` is relative to cluster 2. */
typedef struct stage { struct stage *next; uint32_t sector; uint8_t data[BPS]; } stage_t;
static stage_t *s_stage;
static stage_t *stage_find(uint32_t sector)
{ for (stage_t *p = s_stage; p; p = p->next) if (p->sector == sector) return p; return NULL; }
static uint32_t stage_sector(uint32_t clus, uint32_t intra)
{ return (clus - 2u) * g_sec_per_clus + intra; }
static bool stage_cluster_touched(uint32_t clus)
{
    if (clus < 2) return false;
    uint32_t first = stage_sector(clus, 0);
    for (stage_t *p = s_stage; p; p = p->next)
        if (p->sector >= first && p->sector < first + g_sec_per_clus) return true;
    return false;
}
static bool stage_first_touched(uint32_t clus)
{ return clus >= 2 && stage_find(stage_sector(clus, 0)) != NULL; }
static bool stage_drop(uint32_t sector)
{
    stage_t **link = &s_stage;
    while (*link && (*link)->sector != sector) link = &(*link)->next;
    if (!*link) return false;
    stage_t *st = *link;
    *link = st->next;
    vPortFree(st);
    return true;
}
static void stage_free_all(void)
{ while (s_stage) { stage_t *n = s_stage->next; vPortFree(s_stage); s_stage = n; } }

static bool chain_sector_pos(uint32_t start, uint32_t sector_index,
                             uint32_t *clus, uint32_t *intra)
{
    uint32_t c = start;
    for (uint32_t k = 0; k < sector_index / g_sec_per_clus; k++)
        c = chain_next(c);
    if (c < 2) return false;
    *clus = c;
    *intra = sector_index % g_sec_per_clus;
    return true;
}

static void chain_sector_advance(uint32_t *clus, uint32_t *intra)
{
    if (++*intra == g_sec_per_clus) {
        *intra = 0;
        *clus = chain_next(*clus);
    }
}

/* True once every not-yet-committed data sector up to `size` has been staged,
 * walking the host's real chain rather than assuming contiguous clusters. */
static bool all_staged(uint32_t start, uint32_t committed, uint32_t size)
{
    uint32_t first = committed / BPS;
    uint32_t nsec = size ? (size + BPS - 1u) / BPS : 0;
    uint32_t clus, intra;
    if (first == nsec) return true;
    if (!chain_sector_pos(start, first, &clus, &intra)) return false;
    for (uint32_t k = first; k < nsec; k++) {
        if (clus < 2 || !stage_find(stage_sector(clus, intra))) return false;
        chain_sector_advance(&clus, &intra);
    }
    return true;
}

struct flush_iter_s {
    uint32_t clus, intra;
    uint32_t remaining;
};

static bool flush_next_chunk(void *arg, uint32_t off,
                             const void **data, uint32_t *len)
{
    (void)off;
    struct flush_iter_s *it = arg;
    stage_t *st = stage_find(stage_sector(it->clus, it->intra));
    if (!st || !it->remaining) return false;

    uint32_t n = it->remaining > BPS ? BPS : it->remaining;
    *data = st->data;
    *len = n;
    it->remaining -= n;
    chain_sector_advance(&it->clus, &it->intra);
    return true;
}

/* Stream a fully staged suffix through one open VFS file. Retire complete
 * staging sectors after the stream commits; retain a partial tail for later
 * growth of the same cluster. */
static bool do_flush(struct pending_s *p)
{
    uint32_t first = p->committed / BPS;
    uint32_t clus, intra;
    if (!chain_sector_pos(p->start, first, &clus, &intra)) return false;

    struct flush_iter_s it = {
        .clus = clus,
        .intra = intra,
        .remaining = p->size - p->committed,
    };
    if (vfs_msc_write_chunks(p->vpath, p->committed, p->size,
                             flush_next_chunk, &it) < 0)
        return false;

    uint32_t off = p->committed;
    while (p->size - off >= BPS) {
        if (!stage_drop(stage_sector(clus, intra))) return false;
        off += BPS;
        p->committed = off;
        chain_sector_advance(&clus, &intra);
    }
    return true;
}

/* Record a file the host named in a directory entry, keyed by its first cluster.
 * Size grows through the checkpoints of one upload. A smaller size, or a newly
 * staged first cluster after any prior prefix was committed, starts a
 * same-session overwrite and resets its prefix progress. */
static void pending_add(const char *vpath, uint32_t start, uint32_t size)
{
    if (!s_pending) return;
    for (int i = 0; i < MAX_PENDING; i++)
        if (s_pending[i].start == start) {
            vpath_copy(s_pending[i].vpath, VPATH_MAX, vpath);
            if (size < s_pending[i].size ||
                (s_pending[i].committed > 0 && stage_first_touched(start))) {
                s_pending[i].size = size;
                s_pending[i].committed = 0;
            } else if (size > s_pending[i].size) {
                s_pending[i].size = size;
            }
            return;
        }
    for (int i = 0; i < MAX_PENDING; i++)
        if (!s_pending[i].start) {
            /* A successful sync retires its record and staged partial sector so
             * a long-lived composite MSC mount cannot exhaust MAX_PENDING one
             * completed file at a time. If this is a later append, resume at
             * the persisted file's last sector boundary: FAT writes whole
             * sectors, so the old partial tail (if any) is staged again. A
             * rewrite that touches the first cluster must still start at zero. */
            synfile_t *live = file_of_path(vpath);
            uint32_t committed = 0;
            if (live && live->size < size && !stage_first_touched(start))
                committed = (live->size / BPS) * BPS;
            s_pending[i].start = start;
            s_pending[i].size = size;
            s_pending[i].committed = committed;
            vpath_copy(s_pending[i].vpath, VPATH_MAX, vpath); return;
        }
}

/* A whole directory sector is written back when one entry changes.  Most of its
 * other entries describe unchanged live files and must not become pending writes:
 * they have no staged sectors, so treating them as uploads makes the next SCSI
 * sync report WRITE ERROR even though the one changed file committed correctly.
 * An incremental upload, however, remains pending after its first staged prefix
 * has been retired, so path/cluster membership is also a positive match. */
static bool pending_has(const char *vpath, uint32_t start)
{
    if (!s_pending) return false;
    for (int i = 0; i < MAX_PENDING; i++)
        if (s_pending[i].start == start &&
            strcmp(s_pending[i].vpath, vpath) == 0)
            return true;
    return false;
}

static const char *pending_path_for_start(uint32_t start)
{
    if (!s_pending) return NULL;
    for (int i = 0; i < MAX_PENDING; i++)
        if (s_pending[i].start == start) return s_pending[i].vpath;
    return NULL;
}

static void pending_drop(const char *vpath, uint32_t start)
{
    if (!s_pending) return;
    for (int i = 0; i < MAX_PENDING; i++) {
        struct pending_s *p = &s_pending[i];
        if (p->start != start || strcmp(p->vpath, vpath) != 0) continue;

        uint32_t first = p->committed / BPS;
        uint32_t count = p->size ? (p->size + BPS - 1u) / BPS : 0;
        uint32_t clus, intra;
        if (!chain_sector_pos(p->start, first, &clus, &intra)) {
            p->start = 0;
            return;
        }
        for (uint32_t k = first; k < count; k++) {
            (void)stage_drop(stage_sector(clus, intra));
            chain_sector_advance(&clus, &intra);
        }
        p->start = 0;
        return;
    }
}

/* A metadata-only rename keeps the same clusters, including a retained partial
 * tail used for a later append. Move that pending ownership with the name so a
 * following SYNCHRONIZE CACHE cannot replay the tail under the old path and
 * recreate the rename source. */
static void pending_rename(const char *from, const char *to, uint32_t start)
{
    if (!s_pending) return;
    for (int i = 0; i < MAX_PENDING; i++)
        if (s_pending[i].start == start &&
            strcmp(s_pending[i].vpath, from) == 0) {
            vpath_copy(s_pending[i].vpath, VPATH_MAX, to);
            return;
        }
}

void fatrd_external_forget(const char *path)
{
    if (!path) return;
    fatrd_store_lock();
    int forgotten_dir = dir_index(path);
    if (forgotten_dir >= 0) lfn_cancel_owner(s_dir[forgotten_dir].start);
    dirext_forget(path);
    newdir_forget(path);
    for (int i = 0; i < s_nasn; i++)
        if (strcmp(s_asn[i].vpath, path) == 0) {
            if (s_asn[i].nclus && cluster_is_passthrough(s_asn[i].start))
                ext_free_chain(s_asn[i].start);
            break;
        }
    asn_forget(path);
    if (s_pending) {
        for (int i = 0; i < MAX_PENDING; i++)
            if (s_pending[i].start && strcmp(s_pending[i].vpath, path) == 0) {
                pending_drop(path, s_pending[i].start);
                break;
            }
    }
    for (int i = 0; i < s_ntomb; ) {
        if (strcmp(s_tomb[i].vpath, path) == 0) {
            memmove(s_tomb + i, s_tomb + i + 1,
                    (s_ntomb - i - 1) * sizeof s_tomb[0]);
            s_ntomb--;
        } else {
            i++;
        }
    }
    fatrd_store_unlock();
}

void fatrd_external_rename(const char *from, const char *to)
{
    if (!from || !to) return;
    fatrd_store_lock();
    asn_rename(from, to);
    newdir_rename(from, to);
    if (s_pending) {
        for (int i = 0; i < MAX_PENDING; i++)
            if (s_pending[i].start && strcmp(s_pending[i].vpath, from) == 0) {
                pending_rename(from, to, s_pending[i].start);
                break;
            }
    }
    for (int i = 0; i < s_ntomb; i++)
        if (strcmp(s_tomb[i].vpath, from) == 0)
            vpath_copy(s_tomb[i].vpath, VPATH_MAX, to);
    fatrd_store_unlock();
}

/* Reconcile every recorded file whose next suffix is fully staged. A successful
 * SCSI sync is a durability boundary: retire that record and its retained partial
 * sector. A later append reconstructs its persisted prefix in pending_add(), which
 * keeps a composite device's indefinitely mounted MSC session bounded. */
static bool flush_pending(void)
{
    if (!s_pending) return true;                         /* no writes captured this session */
    bool ok = true;
    for (int i = 0; i < MAX_PENDING; i++) {
        if (!s_pending[i].start) continue;
        if (!all_staged(s_pending[i].start, s_pending[i].committed,
                        s_pending[i].size) || !do_flush(&s_pending[i])) {
            ok = false;
            continue;
        }
        pending_drop(s_pending[i].vpath, s_pending[i].start);
    }
    return ok;
}

/* A directory entry named this file. Only record it (path, first cluster, size) -
 * the actual write happens later in reconcile(), from the complete, quiescent state.
 * The host rewrites this entry several times as the file grows and updates the FAT,
 * so acting now would act on partial state (an intermediate size, an unstaged or
 * not-yet-chained cluster); pending_add keeps the current size, and reconcile flushes
 * once everything is present. A zero-sized entry with a real first cluster marks the
 * beginning of an overwrite and deliberately resets prior progress. */
static void try_flush(const char *vpath, uint32_t start, uint32_t size)
{
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
            if (!vpath_join(out, outlen, prefix, name))
                return false;                 /* path too long to reconstruct */
            *clus = start;
            return true;
        }
        pos += k;
    }
    return false;
}

/* A short entry without its LFN can be either an unchanged synthetic entry
 * whose LFN lives in another sector, or a real short-only rename destination.
 * Prefer the exact alias retained during this mount. After re-enumeration that
 * identity is gone while a host may still write a cached directory sector, so
 * also recognize generated aliases and exact native 8.3 spellings. A long name
 * with no recoverable alias is preserved conservatively until a complete LFN
 * arrives; changing it to a cached host alias would permanently lose its name. */
static bool known_sfn_matches(int dir, const char *known, uint8_t entry_attr,
                              uint32_t entry_start, const uint8_t *entry)
{
    bool saved_match;
    if (asn_sfn_compare(known, entry_start, entry, &saved_match))
        return saved_match;

    for (int n = 0; ; n++) {
        uint8_t attr;
        uint32_t start, size;
        int idx;
        const char *name = dir_item(dir, n, &attr, &start, &size, &idx);
        (void)size;
        if (!name) return false;
        if (start != entry_start || ((attr ^ entry_attr) & 0x10u) != 0 ||
            strcmp(name, leaf(known)) != 0)
            continue;

        uint8_t expected[11];
        short_name(idx, name, expected);
        if ((entry[12] & ASN_SFN_CASE_MASK) == 0) {
            if (memcmp(expected, entry, sizeof expected) == 0)
                return true;

            /* Model insertions can change idx across a fresh mount. Recognize
             * any alias from our FAPxxxxx family when its extension still names
             * this item, rather than mistaking a cached alias for a rename. */
            bool generated = entry[0] == 'F' && entry[1] == 'A' && entry[2] == 'P';
            for (int i = 3; generated && i < 8; i++)
                generated = entry[i] >= '0' && entry[i] <= '9';
            if (generated && memcmp(entry + 8, expected + 8, 3) == 0)
                return true;
        }

        uint8_t native[11], ntres;
        if (sfn_encode_native(name, native, &ntres))
            return memcmp(native, entry, sizeof native) == 0 &&
                   (entry[12] & ASN_SFN_CASE_MASK) == ntres;

        return true;                    /* long name: alias identity was reset */
    }
}

/* Decode a directory sector's worth of entries. LFN entries (attr 0x0F) are
 * accumulated into `lfn`; the following short entry uses the assembled long name
 * (or its own 8.3 name if none). */
static void write_dir(int dir, const char *prefix, uint32_t slot0,
                      uint32_t offset, const uint8_t *buf, uint32_t len)
{
    uint32_t owner = lfn_owner(dir);
    if (owner == 0xFFFFFFFFu) { s_capture_failed = true; return; }
    for (uint32_t i = 0; i < EPS; i++) {
        uint32_t eoff = i * 32;
        if (eoff < offset || eoff + 32 > offset + len) continue;
        const uint8_t *e = buf + (eoff - offset);
        uint8_t c0 = e[0], attr = e[11];

        if (c0 == 0x00 || c0 == 0xE5) {       /* free / deleted directory slot */
            if (c0 == 0x00) lfn_cancel_from(owner, slot0 + i);
            else if (attr != 0x0F) lfn_cancel(owner, slot0 + i);
            /* A 0xE5 marks the name unused, but that is ambiguous (delete vs. the
             * old half of a rename). Tombstone it (deferred): a later new entry at
             * the same cluster claims it as a rename; otherwise it commits as a real
             * delete on the next directory read (tomb_flush). */
            if (attr != 0x0F && c0 == 0xE5) {
                char vp[VPATH_MAX] = "";
                uint32_t clus = e[26] | (e[27] << 8) |
                                (e[20] << 16) | ((uint32_t)e[21] << 24);
                bool resolved = false;
                if (clus >= 2 && (attr & 0x10)) {
                    int dd = dir_of_cluster(clus);
                    const char *nd = (dd >= 0 && s_dir[dd].parent == dir)
                                   ? s_dir[dd].vpath : NULL;
                    resolved = nd && vpath_copy(vp, sizeof vp, nd);
                } else if (clus >= 2) {
                    synfile_t *ff = file_of_cluster_parent(clus, dir);
                    resolved = ff && vpath_copy(vp, sizeof vp, ff->vpath);
                }
                /* FAT leaves a deleted entry's old cluster bytes in place, and
                 * Linux can replay that same 0xE5 slot on an unrelated later
                 * metadata write. Never map such an unresolved, real cluster to
                 * the slot's current occupant: directory insertion/removal makes
                 * slot positions unstable, which would delete an unrelated file.
                 * A zero-length file has no cluster at all, so it is the one case
                 * that must use the slot; require the candidate to still be empty. */
                if (!resolved && clus < 2 &&
                    slot_resolve(dir, slot0 + i, vp, sizeof vp, &clus) &&
                    vfs_size(vp) == 0)
                    resolved = true;
                /* Directory sectors retain old 0xE5 slots indefinitely. If
                 * this mount exposed a different alias for the cluster's live
                 * path, replaying that stale slot is not a new deletion. */
                bool deleted_alias_matches = false;
                if (resolved &&
                    (!asn_deleted_sfn_compare(vp, clus, slot0 + i, e,
                                              &deleted_alias_matches) ||
                     !deleted_alias_matches))
                    resolved = false;
                if (resolved) tomb_add(vp, clus);
            }
            continue;
        }
        if (attr == 0x0F) {                   /* LFN fragment: accumulate for its exact SFN slot */
            int seq = (e[0] & 0x1f) - 1;
            static const int pos[13] = {1,3,5,7,9, 14,16,18,20,22,24, 28,30};
            if (seq >= 0 && seq < 5) {
                /* Ordinal N is physically N entries before the SFN. This remains
                 * true at a sector/cluster boundary, so both write orders converge
                 * on one transaction without relying on checksum uniqueness. */
                uint32_t target = slot0 + i + (uint32_t)seq + 1u;
                int sl = lfn_slot(owner, target, e[13]);
                if (sl < 0) {
                    s_capture_failed = true;
                    continue;
                }
                for (int j = 0; j < LFN_CHARS; j++) {
                    uint16_t ch = e[pos[j]] | (e[pos[j]+1] << 8);
                    int ci = seq * LFN_CHARS + j;
                    if (ch == 0x0000 || ch == 0xFFFF) { if (ci < (int)sizeof(s_lfnp[sl].name) - 1) s_lfnp[sl].name[ci] = '\0'; }
                    else if (ci < (int)sizeof(s_lfnp[sl].name) - 1) s_lfnp[sl].name[ci] = (char)(ch & 0xff);
                }
                if (seq < 5) s_lfnp[sl].state |= (uint8_t)(1u << seq);
                if (e[0] & 0x40) s_lfnp[sl].expected = (uint8_t)(seq + 1);
            }
            continue;
        }
        if (attr & 0x08) continue;            /* volume label */
        if (e[0] == '.') continue;            /* "." ".." */

        uint32_t entry_start = e[26] | (e[27] << 8) |
                               (e[20] << 16) | ((uint32_t)e[21] << 24);
        uint32_t entry_slot = slot0 + i;
        const char *known = NULL;
        bool known_by_slot = false;
        if (attr & 0x10) {
            int kd = dir_of_cluster(entry_start);
            if (kd >= 0 && s_dir[kd].parent == dir) known = s_dir[kd].vpath;
            if (!known) known = newdir_lookup(entry_start);
        } else {
            known = pending_path_for_start(entry_start);
            if (!known) {
                synfile_t *kf = file_of_cluster_parent(entry_start, dir);
                if (kf) known = kf->vpath;
            }
            /* Truncation may move a file to a new cluster without rewriting its
             * LFN. Preserve the live slot only when its saved SFN still matches. */
            if (!known) {
                known = asn_file_for_sfn_slot(dir, entry_slot, e);
                known_by_slot = known != NULL;
            }
        }

        char nm[64];
        uint8_t ck = sfn_cksum(e);
        int sl = lfn_find(owner, entry_slot, ck); /* fragments that name this exact entry */
        if (sl >= 0 && (s_lfnp[sl].state & LFN_HAVE_SHORT) &&
            memcmp(s_lfnp[sl].short_entry, e, 11) != 0) {
            /* The eight-bit checksum collided across two generations of this
             * slot. Never combine the newer SFN with the older fragment mask. */
            lfn_release(sl);
            sl = -1;
        }
        if (sl < 0) lfn_cancel(owner, entry_slot); /* a reused slot supersedes another checksum */
        bool have_lfn = lfn_complete(sl);
        bool known_match = known &&
                           (known_by_slot ||
                            known_sfn_matches(dir, known, attr, entry_start, e));
        /* NTRes lowercase flags are emitted only for a native 8.3 spelling; an
         * SFN backed by an LFN leaves them clear. That lets a lower-case short
         * directory at slot 0 commit in time to route its child writes. */
        bool native_case = (e[12] & ASN_SFN_CASE_MASK) != 0;
        bool might_be_split_lfn = sl >= 0 ||
                                  ((entry_slot % EPS) == 0 && !native_case &&
                                   !s_lfn_ordered_recovery);
        if (!have_lfn && !s_lfn_force_short && might_be_split_lfn) {
            /* This can be a native SFN, but it can equally be the tail of an LFN
             * transaction whose earlier sector has not arrived yet. That ambiguity
             * only exists when a partial transaction already targets this slot, or
             * when the SFN is the sector's first entry. Native SFNs elsewhere must
             * commit immediately so a new short directory can receive child writes
             * before sync. */
            if (sl < 0) sl = lfn_slot(owner, entry_slot, ck);
            if (sl < 0) {
                s_capture_failed = true;
                continue;
            }
            if (!(s_lfnp[sl].state & LFN_HAVE_SHORT)) {
                s_lfn_next_order++;
                if (!s_lfn_next_order) s_lfn_next_order++;
                s_lfnp[sl].order = s_lfn_next_order;
            }
            memcpy(s_lfnp[sl].short_entry, e, sizeof s_lfnp[sl].short_entry);
            s_lfnp[sl].state |= LFN_HAVE_SHORT;
            continue;
        }
        if (have_lfn) {
            strncpy(nm, s_lfnp[sl].name, sizeof(nm)-1);
            nm[sizeof(nm)-1]='\0';
        } else if (known_match) {
            /* A whole-sector rewrite can contain an unchanged short entry whose
             * LFN fragments live in a different sector. A retained/generated
             * alias or exact native spelling proves the stable cluster mapping is
             * authoritative; a different native SFN is rename metadata. */
            strncpy(nm, leaf(known), sizeof(nm)-1);
            nm[sizeof(nm)-1]='\0';
        } else {                                /* no long name: fall back to the 8.3 name */
            sfn_decode_name(e, nm);
        }
        /* A colliding short entry may arrive between fragments of another name.
         * Only the complete matching transaction is consumable/releasable. */
        if (have_lfn) lfn_release(sl);
        if (nm[0] == '\0') continue;

        char vpath[VPATH_MAX];
        if (!vpath_join(vpath, VPATH_MAX, prefix, nm)) continue;

        if (attr & 0x10) {                    /* subdirectory: create it */
            /* Linux writes the whole directory sector when any neighbouring
             * entry changes. Do not replay its unchanged directories through
             * vfs_mkdir(): that API correctly announces an out-of-band media
             * change, but here the mounted MSC host already owns the view. A
             * false UNIT ATTENTION in the middle of an upload makes Linux drop
             * its FAT chain and remount the volume read-only. */
            uint32_t dclus = entry_start;            /* FAT32 full first cluster */
            if (dir_index(vpath) < 0 && vfs_msc_mkdir(vpath) == 0) {
                asn_put(vpath, dclus, 1);
                /* Only a genuinely new directory needs a host-chosen mapping.
                 * Whole-sector rewrites also contain every unchanged built-in;
                 * recording those exhausted MAX_NEWDIR before a real nested
                 * directory on a deep mount such as /mnt/ext0. */
                if (dclus >= 2 && !newdir_lookup(dclus)) {
                    if (s_nnewdir < MAX_NEWDIR) {
                        s_newdir[s_nnewdir].cluster = dclus;
                        s_newdir[s_nnewdir].reconciled = false;
                        strncpy(s_newdir[s_nnewdir].vpath, vpath, VPATH_MAX-1);
                        s_newdir[s_nnewdir].vpath[VPATH_MAX-1] = '\0';
                        s_nnewdir++;
                    } else {
                        s_capture_failed = true;   /* do not ACK children we cannot route */
                    }
                }
            }
            asn_sfn_record(vpath, dclus, e, e[12], slot0 + i);
            continue;
        }

        uint32_t start = entry_start;
        uint32_t size  = e[28] | (e[29]<<8) | (e[30]<<16) | ((uint32_t)e[31]<<24);
        if (start < 2) continue;

        /* A new entry with no staged data that reuses an existing first cluster is a
         * metadata-only rename/move: the file's data didn't move, only its name.
         * The source is either a tombstoned (just-0xE5'd) name or a still-live one.
         * Rename in place; copy across backends. Otherwise it's a real new file. */
        uint32_t nclus = file_nclus(size);
        synfile_t *same_path = file_of_path(vpath);
        bool write_active = stage_cluster_touched(start) || pending_has(vpath, start);

        /* Linux can retain a directory sector in its block cache across the
         * PM3's switch-mode re-enumeration. Cluster assignments are rebuilt on
         * the device, so an unchanged cached entry may no longer resolve by its
         * old first cluster. Recognise it by path + size, adopt that cluster for
         * the rest of this mount, and do not turn it into an impossible pending
         * upload. A real same-size overwrite is not skipped: its staged sector
         * (or prior incremental checkpoint) makes write_active true. */
        if (!write_active && same_path && same_path->size == size &&
            !cluster_is_passthrough(start)) {
            asn_put(vpath, start, nclus);
            asn_sfn_record(vpath, start, e, e[12], slot0 + i);
            continue;
        }

        const char *from = NULL;
        bool from_tomb = false;
        char tvp[VPATH_MAX];
        synfile_t *live = file_of_cluster(start);
        if (tomb_claim(start, tvp)) {
            from = tvp; from_tomb = true;                    /* renamed-from a 0xE5 */
        } else if (!stage_first_touched(start) ||
                   (live && pending_has(live->vpath, start))) {
            /* A partial final sector remains staged after a successful sync so
             * a later append can rewrite that cluster. It still belongs to the
             * live pending file and must not turn a metadata-only rename into a
             * second copied file. An unowned staged cluster is a genuine new
             * write and deliberately cannot claim an unrelated live file. */
            if (live) from = live->vpath;
        }
        if (from && strcmp(from, vpath) != 0) {
            int rrc = vfs_msc_rename(from, vpath);
            if (rrc == 0) {
                asn_rename(from, vpath);          /* destination keeps the source's cluster */
                pending_rename(from, vpath, start);
            } else {                              /* no backend rename (cross-FS / ramfs): copy */
                const uint8_t *data; uint32_t dlen; bool owned;
                if (vfs_read_all(from, &data, &dlen, &owned) == 0) {
                    vfs_msc_write_file(vpath, data, dlen);
                    asn_put(vpath, start, nclus);
                    if (owned) vPortFree((void *)data);
                    /* Complete the move: the host 0xE5'd the source, so drop it
                     * (a live-source match is a copy, not a move - leave it). */
                    if (from_tomb) vfs_msc_remove(from);
                }
            }
        } else if (from && (size == 0 || stage_first_touched(start) ||
                            pending_has(vpath, start))) {
            /* After an incremental prefix commits, build_model() sees the file
             * and file_of_cluster(start) resolves it to this same path. Later
             * directory-size checkpoints are still new staged suffixes, not a
             * metadata-only no-op. Keep extending the pending record. Unchanged
             * neighbours arrive in the same whole-sector directory write, but
             * have neither staged data nor an existing pending record and are
             * deliberately ignored. */
            try_flush(vpath, start, size);
            asn_put(vpath, start, nclus);
        } else {
            if (cluster_is_passthrough(start)) {
                /* Internal-subtree file whose data the host put in the card scratch
                 * (the internal region offers no free clusters). The host writes a
                 * file's data + FAT chain before this final directory entry, so the
                 * data is complete now: read it back off the card and commit it to
                 * the VFS. (An empty file - size 0 - just creates the entry.) */
                uint8_t *b = pvPortMalloc(size ? size : 1);
                if (b) {
                    if (size == 0 || ext_read_file(start, size, b) == 0) {
                        /* This is the mounted host's own write: do not raise
                         * UNIT ATTENTION underneath it. Keep the card scratch
                         * chain (and its assignment) alive until remount so
                         * cached FAT/data reads remain valid. */
                        if (vfs_msc_write_file(vpath, b, size) == 0)
                            asn_put(vpath, start, nclus);
                        else
                            s_capture_failed = true;
                    }
                    else s_capture_failed = true;
                    vPortFree(b);
                } else s_capture_failed = true;
            } else {
                try_flush(vpath, start, size);
                asn_put(vpath, start, nclus);     /* pin the file to the cluster the host chose */
            }
        }
        asn_sfn_record(vpath, start, e, e[12], slot0 + i);
    }
}

/* A host can initialize/populate a new directory's cluster before it publishes
 * that directory's entry in the parent. Until the parent entry arrives there is
 * no cluster -> VFS path route, so the sector is retained (in staging or the
 * external FAT window) but cannot yet be decoded. At sync the s_newdir map is
 * authoritative: read those retained sectors and reconcile their children now. */
static __attribute__((noinline)) bool newdir_reconcile_all(void)
{
    if (!s_nnewdir) return true;
    uint8_t *scratch = NULL;                  /* needed only for card-backed directories */
    bool ok = true;
    for (int n = 0; n < s_nnewdir && ok; n++) {
        if (s_newdir[n].reconciled) continue;
        uint32_t clus = s_newdir[n].cluster;
        for (uint32_t intra = 0; intra < g_sec_per_clus; intra++) {
            build_model();                    /* a prior sector may have added a nested dir */
            int dir = dir_index(s_newdir[n].vpath);
            if (dir < 0) { ok = false; break; }

            const uint8_t *sec = NULL;
            if (cluster_is_passthrough(clus)) {
                if (!scratch) scratch = pvPortMalloc(BPS);
                if (!scratch || fatrd_ext_read(g_pt_data_lba +
                                               (clus - 2u) * g_sec_per_clus + intra,
                                               scratch) != 0) {
                    ok = false;
                    break;
                }
                sec = scratch;
            } else {
                stage_t *st = stage_find(stage_sector(clus, intra));
                if (st) sec = st->data;
            }
            /* An empty new directory need not have a captured synthetic-sector
             * write. Card-backed sectors, however, must remain readable. */
            if (!sec) break;

            uint32_t decode_len = BPS;
            bool end = false;
            for (uint32_t i = 0; i < EPS; i++)
                if (sec[i * 32] == 0x00) {
                    decode_len = (i + 1u) * 32u;
                    end = true;
                    break;
                }
            s_lfn_ordered_recovery = true;
            write_dir(dir, s_dir[dir].vpath, intra * EPS, 0, sec, decode_len);
            s_lfn_ordered_recovery = false;
            if (s_capture_failed) { ok = false; break; }
            if (end) break;
        }
        if (ok) s_newdir[n].reconciled = true;
    }
    vPortFree(scratch);
    return ok;
}

/* Linux can initialize a new directory-extension sector before publishing the
 * FAT link from the old tail. The sector is staged as unknown data at first;
 * once the link is known, replay it in directory order before LFN fallback. */
static __attribute__((noinline)) bool dirext_reconcile_staged(void)
{
    for (int pass = 0; pass < s_ndirext; pass++) {
        int best = -1, best_owner = 0;
        uint32_t best_pos = 0;

        for (int i = 0; i < s_ndirext; i++) {
            if (!stage_cluster_touched(s_dirext[i].cluster)) continue;

            int owner;
            uint32_t pos;
            if (!dirext_position(s_dirext[i].cluster, &owner, &pos))
                return false;
            if (best < 0 || owner < best_owner ||
                (owner == best_owner && pos < best_pos)) {
                best = i;
                best_owner = owner;
                best_pos = pos;
            }
        }
        if (best < 0) break;

        uint32_t clus = s_dirext[best].cluster;
        for (uint32_t intra = 0; intra < g_sec_per_clus; intra++) {
            uint32_t sector = stage_sector(clus, intra);
            stage_t *st = stage_find(sector);
            if (!st) continue;

            uint32_t slot0 = (best_pos * g_sec_per_clus + intra) * EPS;
            s_lfn_ordered_recovery = true;
            if (best_owner == -1)
                write_dir(-1, "", slot0, 0, st->data, BPS);
            else if (best_owner >= 0 && best_owner < s_ndir)
                write_dir(best_owner, s_dir[best_owner].vpath,
                          slot0, 0, st->data, BPS);
            else
                s_capture_failed = true;
            s_lfn_ordered_recovery = false;
            if (s_capture_failed) return false;
            (void)stage_drop(sector);
        }
    }
    return true;
}

static bool newdir_reconcile_needed(void)
{
    for (int i = 0; i < s_nnewdir; i++)
        if (!s_newdir[i].reconciled) return true;
    return false;
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
     * straight to the card so the host maintains its filesystem for us. A
     * directory created below a synthetic VFS path also borrows a card cluster,
     * though: mirror that sector to the card for the mounted host, then decode
     * its entries so files created inside it reach the internal VFS as well. */
    if (cluster_is_passthrough(clus)) {
        int rc = ext_write_sector(g_pt_data_lba +
                                  (clus - 2u) * g_sec_per_clus + intra,
                                  offset, buf, len);
        const char *nd = newdir_lookup(clus);
        if (rc == 0 && nd) {
            int ndi = dir_index(nd);
            if (ndi >= 0)
                write_dir(ndi, nd, intra * EPS, offset, buf, len);
        }
        return rc;
    }

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
        if (ndi >= 0) {
            write_dir(ndi, nd, intra * EPS, offset, buf, len);
            return 0;
        }
    }
    /* A cluster the host chained onto a directory to hold more entries (learned from
     * its FAT write above): route its entry writes to that directory. */
    int own; uint32_t pos;
    if (dirext_position(clus, &own, &pos)) {
        uint32_t slot0 = (pos * g_sec_per_clus + intra) * EPS;
        if (own == -1) write_dir(-1, "", slot0, offset, buf, len);
        else if (own >= 0 && own < s_ndir)
            write_dir(own, s_dir[own].vpath, slot0, offset, buf, len);
        return 0;
    }
    if (offset != 0 || len < BPS) return 0;
    uint32_t sector = rel;                 /* data-sector index relative to cluster 2 */
    stage_t *st = stage_find(sector);
    if (!st) {
        /* Staging keeps the in-flight checkpoint in RAM. Stop retaining sectors
         * near the reserve and let fatrd_sync() report the write failure after
         * the current BOT data phase has been drained. */
        if (xPortGetFreeHeapSize() < STAGE_MIN_FREE + sizeof(stage_t)) {
            s_capture_failed = true;
            return 0;
        }
        st = pvPortMalloc(sizeof(stage_t));
        if (!st) { s_capture_failed = true; return 0; }
        st->sector = sector; st->next = s_stage; s_stage = st;
    }
    memcpy(st->data, buf, BPS);
    return 0;
}

/* Commit any file whose data + directory entry are both complete, without waiting
 * for the next read. The MSC layer calls this on the host's SCSI SYNCHRONIZE CACHE
 * (sent on sync/unmount), so files copied over the mounted drive become durable in
 * LittleFS even if the host never reads the drive again. Staging is kept (idempotent
 * positional writes); it is freed on the next fresh mount. */
bool fatrd_sync(void)
{
    fatrd_store_lock();
    /* At this quiescent durability boundary, an unclaimed tombstone is a real
     * delete. First replay any SFN that was waiting for a cross-sector LFN: a
     * rename destination must be allowed to claim its source tombstone before
     * that source is removed. Then reconcile file data so a delete cannot be
     * recreated from a retained staged tail. */
    if (!s_capture_failed) {
        build_model();                        /* pick up any out-of-band parent rename */
        if (!dirext_reconcile_staged()) s_capture_failed = true;
        if (!s_capture_failed) lfn_pending_flush(true);
    }
    bool ok = !s_capture_failed;
    if (ok) {
        bool settled = false;
        for (int pass = 0; pass <= MAX_NEWDIR; pass++) {
            build_model();                    /* pending parent entries create s_newdir routes */
            ok = newdir_reconcile_all();
            if (!ok) break;
            lfn_pending_flush(true);           /* names split across recovered sectors */
            if (s_capture_failed) { ok = false; break; }
            if (!newdir_reconcile_needed()) { settled = true; break; }
        }
        if (!settled) ok = false;
    }
    if (ok) {
        tomb_flush();
        ok = flush_pending();
    }
    fatrd_store_unlock();
    return ok;
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
    if (s_capture_failed) { fatrd_store_unlock(); return 0; }
    if (!recon_bufs_ensure()) {
        s_capture_failed = true;
        fatrd_store_unlock();
        return 0;
    }
    /* The transfer may span several sectors (CFG_TUD_MSC_EP_BUFSIZE > 512); split it
     * into per-sector captures. `offset` is nonzero only when the EP buffer is
     * smaller than a sector, in which case there is a single partial sector. */
    uint32_t off = offset, cur = lba, remaining = len;
    while (remaining) {
        if (cur >= g_total_sec || off >= BPS) {
            s_capture_failed = true;
            break;
        }
        uint32_t n = BPS - off; if (n > remaining) n = remaining;
        if (write_one_sector(cur, off, buf, n) < 0) {
            s_capture_failed = true;
            break;
        }
        if (s_capture_failed) break;
        buf += n; remaining -= n; off = 0; cur++;
    }
    fatrd_store_unlock();
    /* A capture error is deliberately deferred to fatrd_sync(), after TinyUSB
     * has drained every byte of this WRITE10 and can return a failed CSW without
     * stalling bulk OUT. */
    return 0;
}
