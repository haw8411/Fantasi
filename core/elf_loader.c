#include "elf_loader.h"
#include "elf32.h"

#include "FreeRTOS.h"
#include <string.h>

/* Compiler-runtime helpers an app's codegen may reference (everything else
 * reaches the firmware through the API struct, not symbols). Resolved by name
 * during relocation; taking each address also forces it into the firmware image.
 * An app that references a helper not listed here fails to load with its name. */
extern void *memcpy(void *, const void *, size_t);
extern void *memmove(void *, const void *, size_t);
extern void *memset(void *, int, size_t);
extern void __aeabi_memcpy(void); extern void __aeabi_memcpy4(void); extern void __aeabi_memcpy8(void);
extern void __aeabi_memmove(void); extern void __aeabi_memmove4(void); extern void __aeabi_memmove8(void);
extern void __aeabi_memset(void); extern void __aeabi_memset4(void); extern void __aeabi_memset8(void);
extern void __aeabi_memclr(void); extern void __aeabi_memclr4(void); extern void __aeabi_memclr8(void);
extern void __aeabi_uidiv(void); extern void __aeabi_uidivmod(void);
extern void __aeabi_idiv(void);  extern void __aeabi_idivmod(void);
extern void __aeabi_uldivmod(void); extern void __aeabi_ldivmod(void);
extern void __aeabi_llsl(void); extern void __aeabi_llsr(void); extern void __aeabi_lasr(void);
extern void __aeabi_lmul(void);

static const struct { const char *name; uint32_t addr; } g_helpers[] = {
    { "memcpy",  (uint32_t)(uintptr_t)memcpy  },
    { "memmove", (uint32_t)(uintptr_t)memmove },
    { "memset",  (uint32_t)(uintptr_t)memset  },
    { "__aeabi_memcpy",  (uint32_t)(uintptr_t)__aeabi_memcpy  },
    { "__aeabi_memcpy4", (uint32_t)(uintptr_t)__aeabi_memcpy4 },
    { "__aeabi_memcpy8", (uint32_t)(uintptr_t)__aeabi_memcpy8 },
    { "__aeabi_memmove",  (uint32_t)(uintptr_t)__aeabi_memmove  },
    { "__aeabi_memmove4", (uint32_t)(uintptr_t)__aeabi_memmove4 },
    { "__aeabi_memmove8", (uint32_t)(uintptr_t)__aeabi_memmove8 },
    { "__aeabi_memset",  (uint32_t)(uintptr_t)__aeabi_memset  },
    { "__aeabi_memset4", (uint32_t)(uintptr_t)__aeabi_memset4 },
    { "__aeabi_memset8", (uint32_t)(uintptr_t)__aeabi_memset8 },
    { "__aeabi_memclr",  (uint32_t)(uintptr_t)__aeabi_memclr  },
    { "__aeabi_memclr4", (uint32_t)(uintptr_t)__aeabi_memclr4 },
    { "__aeabi_memclr8", (uint32_t)(uintptr_t)__aeabi_memclr8 },
    { "__aeabi_uidiv",    (uint32_t)(uintptr_t)__aeabi_uidiv    },
    { "__aeabi_uidivmod", (uint32_t)(uintptr_t)__aeabi_uidivmod },
    { "__aeabi_idiv",     (uint32_t)(uintptr_t)__aeabi_idiv     },
    { "__aeabi_idivmod",  (uint32_t)(uintptr_t)__aeabi_idivmod  },
    { "__aeabi_uldivmod", (uint32_t)(uintptr_t)__aeabi_uldivmod },
    { "__aeabi_ldivmod",  (uint32_t)(uintptr_t)__aeabi_ldivmod  },
    { "__aeabi_llsl", (uint32_t)(uintptr_t)__aeabi_llsl },
    { "__aeabi_llsr", (uint32_t)(uintptr_t)__aeabi_llsr },
    { "__aeabi_lasr", (uint32_t)(uintptr_t)__aeabi_lasr },
    { "__aeabi_lmul", (uint32_t)(uintptr_t)__aeabi_lmul },
};

static char s_err[64];
const char *app_load_error(void) { return s_err; }
static int fail(const char *msg) { strncpy(s_err, msg, sizeof(s_err) - 1); s_err[sizeof(s_err)-1]=0; return -1; }

/* Apps that embed Berry scripts reference the VM's public API (be_vm_new,
 * be_tostring, ...); the Berry host provides the real resolver, this weak default
 * (returning "unresolved") keeps non-Berry builds linking. */
__attribute__((weak)) uint32_t berry_resolve_api(const char *name) { (void)name; return 0; }

/* App-facing RFID API resolver (core/rfid/rfid_api.c overrides this where
 * FANTASI_ENABLE_RFID is set); weak default keeps non-RFID builds linking. */
__attribute__((weak)) uint32_t rfid_resolve_api(const char *name) { (void)name; return 0; }

/* App loader API resolver (core/app_run.c provides fantasi_run_module); weak
 * default keeps builds without the app loader linking. */
__attribute__((weak)) uint32_t app_resolve_api(const char *name) { (void)name; return 0; }

static uint32_t resolve_helper(const char *name)
{
    for (unsigned i = 0; i < sizeof(g_helpers)/sizeof(g_helpers[0]); i++)
        if (strcmp(name, g_helpers[i].name) == 0) return g_helpers[i].addr;
    uint32_t a = berry_resolve_api(name);
    if (a) return a;
    a = rfid_resolve_api(name);
    if (a) return a;
    return app_resolve_api(name);
}

/* Set the error string without returning (for the goto-cleanup paths). */
static void seterr(const char *msg) { strncpy(s_err, msg, sizeof(s_err) - 1); s_err[sizeof(s_err)-1] = 0; }

/* Full read at an offset into `dst`; 0 on success. */
static int rd_at(app_read_fn rd, void *ctx, uint32_t off, void *dst, uint32_t len)
{
    return rd(ctx, off, dst, len) == (int32_t)len ? 0 : -1;
}

int app_load(app_read_fn rd, void *ctx, uint32_t total, app_image_t *img)
{
    uint32_t secsz[APP_MAX_SECTIONS] = {0};
    Elf32_Shdr *shdrs = NULL;   /* buffered section header table */
    char       *strtab = NULL;  /* buffered string table */
    uint8_t    *symtab = NULL;  /* buffered symbol table */

    s_err[0] = '\0';
    memset(img, 0, sizeof(*img));

    if (total < sizeof(Elf32_Ehdr)) return fail("truncated");
    Elf32_Ehdr eh;
    if (rd_at(rd, ctx, 0, &eh, sizeof(eh)) != 0) return fail("read error");

    if (eh.e_ident[0] != ELF_MAG0 || eh.e_ident[1] != 'E' ||
        eh.e_ident[2] != 'L' || eh.e_ident[3] != 'F')   return fail("not ELF");
    if (eh.e_ident[4] != ELFCLASS32)                     return fail("not ELF32");
    if (eh.e_ident[5] != ELFDATA2LSB)                    return fail("not LE");
    if (eh.e_type != ET_REL)                             return fail("not relocatable");
    if (eh.e_machine != EM_ARM)                          return fail("not ARM");
    if (eh.e_shentsize != sizeof(Elf32_Shdr))            return fail("bad shentsize");
    if (eh.e_shnum == 0 || eh.e_shnum > APP_MAX_SECTIONS) return fail("too many sections");
    if ((uint64_t)eh.e_shoff + (uint64_t)eh.e_shnum * eh.e_shentsize > total)
        return fail("bad section table");

    /* Buffer the whole section header table once (small; ~40 B/section). */
    uint32_t shsz = (uint32_t)eh.e_shnum * sizeof(Elf32_Shdr);
    shdrs = pvPortMalloc(shsz);
    if (!shdrs) return fail("out of memory");
    if (rd_at(rd, ctx, eh.e_shoff, shdrs, shsz) != 0) { seterr("read error"); goto err; }

    /* Locate the symbol table + its string table. */
    int symtab_idx = -1;
    for (unsigned i = 0; i < eh.e_shnum; i++)
        if (shdrs[i].sh_type == SHT_SYMTAB) symtab_idx = (int)i;
    if (symtab_idx < 0) { seterr("no symtab"); goto err; }
    Elf32_Shdr symtab_sh = shdrs[symtab_idx];
    if (symtab_sh.sh_link >= eh.e_shnum) { seterr("bad symtab link"); goto err; }
    Elf32_Shdr strtab_sh = shdrs[symtab_sh.sh_link];
    if ((uint64_t)symtab_sh.sh_offset + symtab_sh.sh_size > total ||
        (uint64_t)strtab_sh.sh_offset + strtab_sh.sh_size > total) { seterr("bad sym/str range"); goto err; }

    /* Pass 0: lay every SHF_ALLOC section out inside one image block and allocate it.
     *
     * Both of these matter for heap fragmentation, which is what limits how much a loaded
     * module can then allocate for itself on a small heap:
     *
     *  - One block, not one per section. Per-section allocation scattered a module's
     *    text/rodata/bss (plus a heap_4 header each) through the free space it was carving
     *    them out of; a single block leaves the remainder contiguous.
     *  - Allocated before strtab/symtab (which only relocation needs). Those are freed
     *    before returning, so allocating them after the image means their space coalesces
     *    back into the tail free region instead of being stranded as a hole in front of the
     *    resident image - which is what split the heap into two useless halves.
     *
     * Sections are packed at their own sh_addralign (capped at portBYTE_ALIGNMENT, which
     * pvPortMalloc guarantees for the base), so the layout is exactly as strict as before. */
    uint32_t imgsz = 0;
    uint32_t secoff[APP_MAX_SECTIONS] = {0};
    for (unsigned i = 0; i < eh.e_shnum; i++) {
        Elf32_Shdr *sh = &shdrs[i];
        if (!(sh->sh_flags & SHF_ALLOC) || sh->sh_size == 0) continue;
        if (sh->sh_addralign > portBYTE_ALIGNMENT) { seterr("over-aligned"); goto err; }
        uint32_t a = sh->sh_addralign ? sh->sh_addralign : 1;
        imgsz = (imgsz + a - 1) & ~(a - 1);
        secoff[i] = imgsz;
        imgsz += sh->sh_size;
        if (sh->sh_type != SHT_NOBITS && (uint64_t)sh->sh_offset + sh->sh_size > total) {
            seterr("bad section"); goto err;
        }
    }
    if (imgsz) {
        img->base = pvPortMalloc(imgsz);
        if (!img->base) { seterr("out of memory"); goto err; }
    }

    /* Buffer strtab + symtab so relocation lookups stay in RAM (no per-symbol
     * file reads). These are freed before return; only the loaded image stays. */
    uint32_t strsz = strtab_sh.sh_size, symsz = symtab_sh.sh_size;
    strtab = pvPortMalloc(strsz ? strsz : 1);
    symtab = pvPortMalloc(symsz ? symsz : 1);
    if (!strtab || !symtab) { seterr("out of memory"); goto err; }
    if (rd_at(rd, ctx, strtab_sh.sh_offset, strtab, strsz) != 0 ||
        rd_at(rd, ctx, symtab_sh.sh_offset, symtab, symsz) != 0) { seterr("read error"); goto err; }
    unsigned nsym = symsz / sizeof(Elf32_Sym);

    /* Pass 1: point each section at its slot in the image block and stream it
     * straight from the file into that RAM (no whole-ELF buffer). */
    for (unsigned i = 0; i < eh.e_shnum; i++) {
        Elf32_Shdr *sh = &shdrs[i];
        if (!(sh->sh_flags & SHF_ALLOC) || sh->sh_size == 0) continue;
        void *mem = (uint8_t *)img->base + secoff[i];
        img->sec[i] = mem;
        secsz[i] = sh->sh_size;
        if (sh->sh_type == SHT_NOBITS) {
            memset(mem, 0, sh->sh_size);
        } else {
            if (rd_at(rd, ctx, sh->sh_offset, mem, sh->sh_size) != 0) { seterr("read error"); goto err; }
        }
    }

    /* Pass 2: apply relocations, one SHT_REL section at a time into a temp buffer. */
    for (unsigned i = 0; i < eh.e_shnum; i++) {
        Elf32_Shdr *rsh = &shdrs[i];
        if (rsh->sh_type != SHT_REL) continue;
        unsigned tgt = rsh->sh_info;
        if (tgt >= eh.e_shnum || !img->sec[tgt]) continue;   /* reloc for a non-loaded section */
        if (rsh->sh_entsize != sizeof(Elf32_Rel)) { seterr("bad rel entsize"); goto err; }
        if ((uint64_t)rsh->sh_offset + rsh->sh_size > total) { seterr("bad rel range"); goto err; }

        uint8_t *relbuf = pvPortMalloc(rsh->sh_size ? rsh->sh_size : 1);
        if (!relbuf) { seterr("out of memory"); goto err; }
        if (rd_at(rd, ctx, rsh->sh_offset, relbuf, rsh->sh_size) != 0) {
            vPortFree(relbuf); seterr("read error"); goto err;
        }

        unsigned nrel = rsh->sh_size / sizeof(Elf32_Rel);
        for (unsigned r = 0; r < nrel; r++) {
            Elf32_Rel rel;
            memcpy(&rel, relbuf + r * sizeof(Elf32_Rel), sizeof(rel));
            unsigned type = ELF32_R_TYPE(rel.r_info);
            unsigned symi = ELF32_R_SYM(rel.r_info);
            /* R_ARM_V4BX marks BX instructions so a final link *could* rewrite
             * them for pre-ARMv4T cores. AT91SAM7S is ARMv4T (BX is valid), so
             * it is a no-op for us - skip without touching the instruction. */
            if (type == R_ARM_V4BX) continue;
            if (type != R_ARM_ABS32 && type != R_ARM_TARGET1) {
                vPortFree(relbuf); seterr("unsupported reloc"); goto err;
            }
            if (symi >= nsym) { vPortFree(relbuf); seterr("bad sym index"); goto err; }

            Elf32_Sym sym;
            memcpy(&sym, symtab + (size_t)symi * sizeof(Elf32_Sym), sizeof(sym));
            uint32_t S;
            if (sym.st_shndx == SHN_UNDEF) {
                if (sym.st_name >= strsz) { vPortFree(relbuf); seterr("bad sym name"); goto err; }
                const char *nm = strtab + sym.st_name;
                S = resolve_helper(nm);
                if (!S) { vPortFree(relbuf); seterr("undef: ");
                          strncat(s_err, nm, sizeof(s_err) - 9); goto err; }
            } else if (sym.st_shndx == SHN_ABS) {
                S = sym.st_value;
            } else if (sym.st_shndx < eh.e_shnum && img->sec[sym.st_shndx]) {
                S = (uint32_t)(uintptr_t)img->sec[sym.st_shndx] + sym.st_value;
            } else {
                vPortFree(relbuf); seterr("bad reloc sym"); goto err;
            }

            if (rel.r_offset + 4 > secsz[tgt]) { vPortFree(relbuf); seterr("reloc out of range"); goto err; }
            uint8_t *loc = (uint8_t *)img->sec[tgt] + rel.r_offset;
            uint32_t v; memcpy(&v, loc, 4); v += S; memcpy(loc, &v, 4);
        }
        vPortFree(relbuf);
    }

    /* Resolve the entry point: the global symbol "app_main". */
    for (unsigned i = 0; i < nsym; i++) {
        Elf32_Sym sym;
        memcpy(&sym, symtab + (size_t)i * sizeof(Elf32_Sym), sizeof(sym));
        if (sym.st_name == 0 || sym.st_name >= strsz) continue;
        if (strcmp(strtab + sym.st_name, "app_main") != 0) continue;
        if (sym.st_shndx >= eh.e_shnum || !img->sec[sym.st_shndx]) break;
        img->entry = (uint32_t)(uintptr_t)img->sec[sym.st_shndx] + sym.st_value;
        vPortFree(shdrs); vPortFree(strtab); vPortFree(symtab);
        return 0;
    }

    seterr("no app_main");
err:
    if (shdrs)  vPortFree(shdrs);
    if (strtab) vPortFree(strtab);
    if (symtab) vPortFree(symtab);
    app_unload(img);
    return -1;
}

void app_unload(app_image_t *img)
{
    /* One allocation covers every section (see app_load Pass 0), so sec[] entries are
     * interior pointers - free only the base. */
    if (img->base) { vPortFree(img->base); img->base = NULL; }
    for (int i = 0; i < APP_MAX_SECTIONS; i++) img->sec[i] = NULL;
    img->entry = 0;
}
