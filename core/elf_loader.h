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

/* Load `elf` (length `len`) into freshly allocated RAM. Returns 0 on success
 * with *img populated, or a negative error code. The source buffer is not
 * retained (sections are copied), so the caller may free it after loading. */
int  app_load(const uint8_t *elf, uint32_t len, app_image_t *img);

/* Free every allocation made by app_load. */
void app_unload(app_image_t *img);

/* Human-readable reason for the last app_load failure (valid until the next call). */
const char *app_load_error(void);

#endif /* CORE_ELF_LOADER_H */
