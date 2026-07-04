#include "fat_ramdisk.h"
#include "../../core/vfs.h"

#include "FreeRTOS.h"
#include <string.h>
#include <stdio.h>

/* Synthetic FAT16 (with VFAT long filenames) view of the whole VFS. The root
 * mirrors the LittleFS root (files + one level of subdirectories, e.g. /apps);
 * targets with a RAM filesystem also get a synthetic /ramfs folder. Reads
 * synthesize boot/FAT/directories/file-data from the VFS; writes parse directory
 * entries (create/delete) and stage data sectors, flushing a complete file
 * through the VFS (which routes /ramfs to RAM and everything else to flash).
 * Long/mixed-case names round-trip via VFAT LFN entries; each item also carries
 * a generated unique 8.3 short name the LFN entries hang off. */

/* ---- FAT16 geometry (1 sector/cluster) ----
 *   lba 0      boot   |  1..33 FAT  |  34..65 root dir  |  66.. data (cluster 2) */
#define BPS         512u
#define RSVD        1u
#define NFAT        1u
#define FATSEC      33u
#define ROOTSEC     32u
#define FAT_LBA     RSVD
#define ROOT_LBA    (RSVD + NFAT * FATSEC)   /* 34 */
#define DATA_LBA    (ROOT_LBA + ROOTSEC)     /* 66 */
#define EPS         16u                      /* dir entries per sector */
#define LBA_TO_CLUS(l) ((l) - DATA_LBA + 2u)

#define MAX_DIRS   6
#define MAX_FILES  40
#define VPATH_MAX  96
#define LFN_CHARS  13                        /* name chars per LFN entry */

typedef struct { char vpath[VPATH_MAX]; uint32_t start, nclus; } syndir_t;
typedef struct { char vpath[VPATH_MAX]; uint32_t size, start, nclus; int parent; } synfile_t;

static syndir_t  s_dir[MAX_DIRS];
static synfile_t s_file[MAX_FILES];
static int       s_ndir, s_nfile;

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
        if (s_ndir >= MAX_DIRS) return;
        snprintf(s_dir[s_ndir].vpath, VPATH_MAX, "/%s", name);
        s_ndir++;
    } else {
        if (s_nfile >= MAX_FILES) return;
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
    if (is_dir || s_nfile >= MAX_FILES) return;
    snprintf(s_file[s_nfile].vpath, VPATH_MAX, "%s/%s", c->base, name);
    s_file[s_nfile].size = size;
    s_file[s_nfile].parent = c->dir;
    s_nfile++;
}

/* number of entries in directory `dir` (-1 = root): leading specials + items */
static uint32_t dir_entry_count(int dir)
{
    uint32_t n = (dir < 0) ? 1 : 2;          /* root: vol label; subdir: ".",".." */
    for (int d = 0; d < s_ndir; d++)
        if (dir < 0) n += name_entries(leaf(s_dir[d].vpath));
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

void fatrd_invalidate(void) { s_fs_gen++; }

static void build_model(void)
{
    if (s_have_model && s_built_gen == s_fs_gen) return;   /* cache still valid */
    s_ndir = 0; s_nfile = 0;
    vfs_list("/", root_cb, NULL);   /* includes the synthetic /ramfs mount */
    for (int d = 0; d < s_ndir; d++) {
        struct sub_ctx c = { d, s_dir[d].vpath };
        vfs_list(s_dir[d].vpath, sub_cb, &c);
    }
    uint32_t clus = 2;
    for (int d = 0; d < s_ndir; d++) {
        uint32_t ents = dir_entry_count(d);
        s_dir[d].start = clus;
        s_dir[d].nclus = (ents + EPS - 1) / EPS;
        if (!s_dir[d].nclus) s_dir[d].nclus = 1;
        clus += s_dir[d].nclus;
    }
    for (int i = 0; i < s_nfile; i++) {
        s_file[i].start = clus;
        s_file[i].nclus = s_file[i].size ? (s_file[i].size + BPS - 1) / BPS : 1;
        clus += s_file[i].nclus;
    }
    s_built_gen = s_fs_gen;
    s_have_model = true;
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
static struct { uint32_t cluster; char vpath[VPATH_MAX]; } s_newdir[MAX_NEWDIR];
static int s_nnewdir;
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
    put16(e + 26, (uint16_t)start);
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
    s[0]=0xEB; s[1]=0x3C; s[2]=0x90; memcpy(s+3,"MSDOS5.0",8);
    put16(s+11,BPS); s[13]=1; put16(s+14,RSVD); s[16]=NFAT;
    put16(s+17, ROOTSEC*EPS); put16(s+19, FATRD_SECTOR_COUNT);
    s[21]=0xF8; put16(s+22,FATSEC); put16(s+24,32); put16(s+26,8);
    s[36]=0x80; s[38]=0x29; put32(s+39,0xFA17A511);
    memcpy(s+43,"Fantasi    ",11); memcpy(s+54,"FAT16   ",8);
    s[510]=0x55; s[511]=0xAA;
}

static uint16_t fat_entry(uint32_t e)
{
    if (e == 0) return 0xFFF8;
    if (e == 1) return 0xFFFF;
    for (int d = 0; d < s_ndir; d++)
        if (e >= s_dir[d].start && e < s_dir[d].start + s_dir[d].nclus)
            return (e == s_dir[d].start + s_dir[d].nclus - 1) ? 0xFFFF : (uint16_t)(e+1);
    synfile_t *f = file_of_cluster(e);
    if (f) return (e == f->start + f->nclus - 1) ? 0xFFFF : (uint16_t)(e+1);
    return 0x0000;
}

static void synth_fat(uint32_t sec, uint8_t s[BPS])
{
    uint32_t first = sec * (BPS/2);
    for (uint32_t i = 0; i < BPS/2; i++) put16(s + i*2, fat_entry(first + i));
}

/* Item lookup within a directory: returns the n-th item (0-based) as name/attr/
 * start/size, or NULL when out of range. idx_out gets the item index in the dir. */
static const char *dir_item(int dir, int n, uint8_t *attr, uint32_t *start, uint32_t *size, int *idx_out)
{
    int k = 0;
    if (dir < 0) {
        for (int d = 0; d < s_ndir; d++, k++)
            if (k == n) { *attr=0x10; *start=s_dir[d].start; *size=0; *idx_out=k; return leaf(s_dir[d].vpath); }
        for (int i = 0; i < s_nfile; i++) if (s_file[i].parent == -1) {
            if (k == n) { *attr=0x20; *start=s_file[i].start; *size=s_file[i].size; *idx_out=k; return leaf(s_file[i].vpath); }
            k++;
        }
    } else {
        for (int i = 0; i < s_nfile; i++) if (s_file[i].parent == dir) {
            if (k == n) { *attr=0x20; *start=s_file[i].start; *size=s_file[i].size; *idx_out=k; return leaf(s_file[i].vpath); }
            k++;
        }
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
    if (dir >= 0 && g == 1) { uint8_t nm[11]; memset(nm,' ',11); nm[0]='.'; nm[1]='.'; short_entry(e, nm, 0x10, 0, 0); return; }

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

int fatrd_read(uint32_t lba, uint32_t offset, void *buf, uint32_t len)
{
    if (lba >= FATRD_SECTOR_COUNT) return -1;
    build_model();

    uint8_t sec[BPS];
    if (lba == 0) { s_nnewdir = 0; synth_boot(sec); }   /* fresh mount: drop stale dir routes */
    else if (lba < FAT_LBA + NFAT*FATSEC) synth_fat((lba - FAT_LBA) % FATSEC, sec);
    else if (lba < ROOT_LBA + ROOTSEC)    synth_dir(-1, (lba - ROOT_LBA) * EPS, sec);
    else {
        uint32_t clus = LBA_TO_CLUS(lba);
        int d = dir_of_cluster(clus);
        if (d >= 0) synth_dir(d, (clus - s_dir[d].start) * EPS, sec);
        else {
            memset(sec, 0, BPS);
            synfile_t *f = file_of_cluster(clus);
            if (f) vfs_pread(f->vpath, (clus - f->start) * BPS, sec, BPS);
        }
    }
    if (offset >= BPS) return -1;
    if (offset + len > BPS) len = BPS - offset;
    memcpy(buf, sec + offset, len);
    return 0;
}

/* ---- WRITE (capture) ---- */

typedef struct stage { struct stage *next; uint32_t cluster; uint8_t data[BPS]; } stage_t;
static stage_t *s_stage;
static stage_t *stage_find(uint32_t c)
{ for (stage_t *p = s_stage; p; p = p->next) if (p->cluster == c) return p; return NULL; }

/* Map a directory slot (global 32-byte entry index `g`) back to the full vpath of
 * the item whose LFN+short entry chain covers it - the inverse of emit_entry's
 * layout. The host deletes an entry by writing 0xE5 over the exact slot it read
 * from synth_dir, so resolving the slot against the *current* model is reliable
 * regardless of when the file was created (no creation-time slot bookkeeping). */
static const char *slot_vpath(int dir, uint32_t g, char *out, uint32_t outlen)
{
    uint32_t lead = (dir < 0) ? 1 : 2;      /* volume label / "." ".." */
    if (g < lead) return NULL;
    uint32_t pos = lead;
    for (int n = 0; ; n++) {
        uint8_t attr; uint32_t start, size; int idx;
        const char *name = dir_item(dir, n, &attr, &start, &size, &idx);
        if (!name) return NULL;
        uint32_t k = name_entries(name);
        if (g < pos + k) {
            const char *prefix = (dir < 0) ? "" : s_dir[dir].vpath;
            snprintf(out, outlen, "%s/%s", prefix, name);
            return out;
        }
        pos += k;
    }
}

static void try_flush(const char *vpath, uint32_t start, uint32_t size)
{
    uint32_t nclus = size ? (size + BPS - 1) / BPS : 0;
    for (uint32_t k = 0; k < nclus; k++) if (!stage_find(start + k)) return;

    uint8_t *buf = pvPortMalloc(size ? size : 1);

    /* Assemble the file while releasing each staged cluster as it is consumed.
     * Freeing as we go (rather than after the write) keeps peak heap to roughly
     * one file buffer + the ramfs copy instead of also holding every staged
     * sector - the difference between fitting and OOM on PM3's ~11 KB heap.
     * The stages are unlinked unconditionally so a failed (OOM) flush can't pin
     * heap; on success vfs_write_file copies `buf` into the destination FS. */
    for (uint32_t k = 0; k < nclus; k++) {
        stage_t *st = NULL;
        for (stage_t **pp = &s_stage; *pp; pp = &(*pp)->next)
            if ((*pp)->cluster == start + k) { st = *pp; *pp = st->next; break; }
        if (!st) continue;
        if (buf) {
            uint32_t off = k*BPS, n = size - off; if (n > BPS) n = BPS;
            if (off < size) memcpy(buf + off, st->data, n);
        }
        vPortFree(st);
    }

    if (buf) { vfs_write_file(vpath, buf, size); vPortFree(buf); }
}

/* Decode a directory sector's worth of entries. LFN entries (attr 0x0F) are
 * accumulated into `lfn`; the following short entry uses the assembled long name
 * (or its own 8.3 name if none). */
static void write_dir(int dir, const char *prefix, uint32_t slot0,
                      uint32_t offset, const uint8_t *buf, uint32_t len)
{
    static char lfn[64];
    static bool have_lfn;
    for (uint32_t i = 0; i < EPS; i++) {
        uint32_t eoff = i * 32;
        if (eoff < offset || eoff + 32 > offset + len) continue;
        const uint8_t *e = buf + (eoff - offset);
        uint8_t c0 = e[0], attr = e[11];

        if (c0 == 0x00 || c0 == 0xE5) {       /* free/deleted */
            /* attr==0x0F marks a deleted LFN fragment; act only on the short
             * entry (attr still holds the file/dir attrs) so each item is
             * removed once. Resolve the slot against the live model. */
            if (attr != 0x0F) {
                have_lfn = false;
                if (c0 == 0xE5) {
                    char vp[VPATH_MAX];
                    const char *v = slot_vpath(dir, slot0 + i, vp, sizeof(vp));
                    if (v) vfs_remove(v);
                }
            }
            continue;
        }
        if (attr == 0x0F) {                   /* LFN fragment */
            int seq = (e[0] & 0x1f) - 1;
            static const int pos[13] = {1,3,5,7,9, 14,16,18,20,22,24, 28,30};
            if (seq >= 0 && seq < 20) {
                for (int j = 0; j < LFN_CHARS; j++) {
                    uint16_t ch = e[pos[j]] | (e[pos[j]+1] << 8);
                    int ci = seq * LFN_CHARS + j;
                    if (ch == 0x0000 || ch == 0xFFFF) { if (ci < (int)sizeof(lfn)) lfn[ci] = '\0'; }
                    else if (ci < (int)sizeof(lfn) - 1) lfn[ci] = (char)(ch & 0xff);
                }
                have_lfn = true;
            }
            continue;
        }
        if (attr & 0x08) { have_lfn = false; continue; }   /* volume label */
        if (e[0] == '.') { have_lfn = false; continue; }   /* "." ".." */

        char nm[64];
        if (have_lfn && lfn[0]) { strncpy(nm, lfn, sizeof(nm)-1); nm[sizeof(nm)-1]='\0'; }
        else {                                /* fall back to the 8.3 name */
            int o = 0;
            for (int k = 0; k < 8 && e[k] != ' '; k++) { char ch=e[k]; nm[o++]=(ch>='A'&&ch<='Z')?ch+32:ch; }
            if (e[8] != ' ') { nm[o++]='.'; for (int k=0;k<3&&e[8+k]!=' ';k++){char ch=e[8+k]; nm[o++]=(ch>='A'&&ch<='Z')?ch+32:ch;} }
            nm[o] = '\0';
        }
        have_lfn = false; lfn[0] = '\0';
        if (nm[0] == '\0') continue;

        char vpath[VPATH_MAX];
        snprintf(vpath, VPATH_MAX, "%s/%s", prefix, nm);

        if (attr & 0x10) {                    /* subdirectory: create it */
            vfs_mkdir(vpath);
            uint32_t dclus = e[26] | (e[27] << 8);   /* cluster the host chose */
            if (dclus >= 2 && !newdir_lookup(dclus) && s_nnewdir < MAX_NEWDIR) {
                s_newdir[s_nnewdir].cluster = dclus;
                strncpy(s_newdir[s_nnewdir].vpath, vpath, VPATH_MAX-1);
                s_newdir[s_nnewdir].vpath[VPATH_MAX-1] = '\0';
                s_nnewdir++;
            }
            continue;
        }

        uint32_t start = e[26] | (e[27] << 8);
        uint32_t size  = e[28] | (e[29]<<8) | (e[30]<<16) | ((uint32_t)e[31]<<24);
        if (start < 2) continue;

        try_flush(vpath, start, size);
    }
}

int fatrd_write(uint32_t lba, uint32_t offset, const uint8_t *buf, uint32_t len)
{
    if (lba >= FATRD_SECTOR_COUNT) return -1;
    if (lba == 0 || lba < FAT_LBA + NFAT*FATSEC) return 0;

    build_model();

    if (lba < ROOT_LBA + ROOTSEC) {
        uint32_t sec = lba - ROOT_LBA;
        write_dir(-1, "", sec * EPS, offset, buf, len);
        return 0;
    }
    uint32_t clus = LBA_TO_CLUS(lba);
    int d = dir_of_cluster(clus);
    if (d >= 0) {
        uint32_t slot0 = (clus - s_dir[d].start) * EPS;
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
    if (offset != 0 || len < BPS) return 0;
    stage_t *st = stage_find(clus);
    if (!st) { st = pvPortMalloc(sizeof(stage_t)); if (!st) return -1;
               st->cluster = clus; st->next = s_stage; s_stage = st; }
    memcpy(st->data, buf, BPS);
    return 0;
}
