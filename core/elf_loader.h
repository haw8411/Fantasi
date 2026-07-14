/* elf_loader - loads a freestanding, relocatable ARM object (ET_REL) into RAM,
 * applies its relocations, and resolves app_main. Apps are built with
 * -mword-relocations -mlong-calls so the only relocation type is R_ARM_ABS32 and
 * the only external symbols are a few compiler-runtime helpers, which the loader
 * resolves against the firmware. No PIC/GOT/Thumb-branch handling is required. */
#ifndef CORE_ELF_LOADER_H
#define CORE_ELF_LOADER_H

#include <stdint.h>

#define APP_MAX_SECTIONS 32

typedef struct {
    void    *sec[APP_MAX_SECTIONS];   /* allocated base per ELF section (NULL if none) */
    uint32_t entry;                   /* app_main runtime address (Thumb bit set) */
} app_image_t;

/* Random-access read of the ELF image: copy `len` bytes at `off` into `dst`.
 * Returns bytes read (== len on success) or <0 on error. The loader reads the
 * image through this callback (backed by vfs_pread), so peak load RAM is the
 * loaded image plus small metadata - not the entire ELF file in RAM. */
typedef int32_t (*app_read_fn)(void *ctx, uint32_t off, void *dst, uint32_t len);

/* Load the ELF read via (read, ctx), total size `total`, into freshly allocated
 * RAM. Returns 0 on success with *img populated, or a negative error code. Only
 * the loadable sections are retained; the loader buffers just the small metadata
 * (section headers, symtab, strtab) during the load and frees it before return. */
int  app_load(app_read_fn read, void *ctx, uint32_t total, app_image_t *img);

/* Free every allocation made by app_load. */
void app_unload(app_image_t *img);

/* Human-readable reason for the last app_load failure (valid until the next call). */
const char *app_load_error(void);

#endif /* CORE_ELF_LOADER_H */
