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

static uint32_t resolve_helper(const char *name)
{
    for (unsigned i = 0; i < sizeof(g_helpers)/sizeof(g_helpers[0]); i++)
        if (strcmp(name, g_helpers[i].name) == 0) return g_helpers[i].addr;
    return 0;
}

/* Read a section header (memcpy avoids any unaligned-struct concern). */
static void read_shdr(const uint8_t *elf, const Elf32_Ehdr *eh, unsigned i, Elf32_Shdr *out)
{
    memcpy(out, elf + eh->e_shoff + (size_t)i * eh->e_shentsize, sizeof(*out));
}

int app_load(const uint8_t *elf, uint32_t len, app_image_t *img)
{
    uint32_t secsz[APP_MAX_SECTIONS] = {0};

    s_err[0] = '\0';
    memset(img, 0, sizeof(*img));

    if (len < sizeof(Elf32_Ehdr)) return fail("truncated");
    Elf32_Ehdr eh;
    memcpy(&eh, elf, sizeof(eh));

    if (eh.e_ident[0] != ELF_MAG0 || eh.e_ident[1] != 'E' ||
        eh.e_ident[2] != 'L' || eh.e_ident[3] != 'F')   return fail("not ELF");
    if (eh.e_ident[4] != ELFCLASS32)                     return fail("not ELF32");
    if (eh.e_ident[5] != ELFDATA2LSB)                    return fail("not LE");
    if (eh.e_type != ET_REL)                             return fail("not relocatable");
    if (eh.e_machine != EM_ARM)                          return fail("not ARM");
    if (eh.e_shentsize != sizeof(Elf32_Shdr))            return fail("bad shentsize");
    if (eh.e_shnum > APP_MAX_SECTIONS)                   return fail("too many sections");
    if ((uint64_t)eh.e_shoff + (uint64_t)eh.e_shnum * eh.e_shentsize > len)
        return fail("bad section table");

    /* Locate the symbol table + its string table. */
    int symtab_idx = -1;
    Elf32_Shdr symtab_sh = {0}, strtab_sh = {0};
    for (unsigned i = 0; i < eh.e_shnum; i++) {
        Elf32_Shdr sh; read_shdr(elf, &eh, i, &sh);
        if (sh.sh_type == SHT_SYMTAB) { symtab_idx = (int)i; symtab_sh = sh; }
    }
    if (symtab_idx < 0) return fail("no symtab");
    if (symtab_sh.sh_link >= eh.e_shnum) return fail("bad symtab link");
    read_shdr(elf, &eh, symtab_sh.sh_link, &strtab_sh);
    if ((uint64_t)symtab_sh.sh_offset + symtab_sh.sh_size > len ||
        (uint64_t)strtab_sh.sh_offset + strtab_sh.sh_size > len)
        return fail("bad sym/str range");
    const char *strtab = (const char *)(elf + strtab_sh.sh_offset);

    /* Pass 1: allocate and load every SHF_ALLOC section. */
    for (unsigned i = 0; i < eh.e_shnum; i++) {
        Elf32_Shdr sh; read_shdr(elf, &eh, i, &sh);
        if (!(sh.sh_flags & SHF_ALLOC) || sh.sh_size == 0) continue;
        if (sh.sh_addralign > portBYTE_ALIGNMENT) { app_unload(img); return fail("over-aligned"); }
        void *mem = pvPortMalloc(sh.sh_size);
        if (!mem) { app_unload(img); return fail("out of memory"); }
        if (sh.sh_type == SHT_NOBITS) {
            memset(mem, 0, sh.sh_size);
        } else {
            if ((uint64_t)sh.sh_offset + sh.sh_size > len) { app_unload(img); return fail("bad section"); }
            memcpy(mem, elf + sh.sh_offset, sh.sh_size);
        }
        img->sec[i] = mem;
        secsz[i] = sh.sh_size;
    }

    /* Pass 2: apply relocations. */
    for (unsigned i = 0; i < eh.e_shnum; i++) {
        Elf32_Shdr rsh; read_shdr(elf, &eh, i, &rsh);
        if (rsh.sh_type != SHT_REL) continue;
        unsigned tgt = rsh.sh_info;
        if (tgt >= eh.e_shnum || !img->sec[tgt]) continue;   /* reloc for a non-loaded section */
        if (rsh.sh_entsize != sizeof(Elf32_Rel)) { app_unload(img); return fail("bad rel entsize"); }
        if ((uint64_t)rsh.sh_offset + rsh.sh_size > len) { app_unload(img); return fail("bad rel range"); }

        unsigned nrel = rsh.sh_size / sizeof(Elf32_Rel);
        for (unsigned r = 0; r < nrel; r++) {
            Elf32_Rel rel;
            memcpy(&rel, elf + rsh.sh_offset + r * sizeof(Elf32_Rel), sizeof(rel));
            unsigned type = ELF32_R_TYPE(rel.r_info);
            unsigned symi = ELF32_R_SYM(rel.r_info);
            /* R_ARM_V4BX marks BX instructions so a final link *could* rewrite
             * them for pre-ARMv4T cores. AT91SAM7S is ARMv4T (BX is valid), so
             * it is a no-op for us - skip without touching the instruction. */
            if (type == R_ARM_V4BX) continue;
            if (type != R_ARM_ABS32 && type != R_ARM_TARGET1) {
                app_unload(img); return fail("unsupported reloc");
            }

            Elf32_Sym sym;
            memcpy(&sym, elf + symtab_sh.sh_offset + symi * sizeof(Elf32_Sym), sizeof(sym));
            uint32_t S;
            if (sym.st_shndx == SHN_UNDEF) {
                const char *nm = strtab + sym.st_name;
                S = resolve_helper(nm);
                if (!S) { app_unload(img); strncpy(s_err, "undef: ", 8);
                          strncat(s_err, nm, sizeof(s_err)-9); return -1; }
            } else if (sym.st_shndx == SHN_ABS) {
                S = sym.st_value;
            } else if (sym.st_shndx < eh.e_shnum && img->sec[sym.st_shndx]) {
                S = (uint32_t)(uintptr_t)img->sec[sym.st_shndx] + sym.st_value;
            } else {
                app_unload(img); return fail("bad reloc sym");
            }

            if (rel.r_offset + 4 > secsz[tgt]) { app_unload(img); return fail("reloc out of range"); }
            uint8_t *loc = (uint8_t *)img->sec[tgt] + rel.r_offset;
            uint32_t v; memcpy(&v, loc, 4); v += S; memcpy(loc, &v, 4);
        }
    }

    /* Resolve the entry point: the global symbol "app_main". */
    unsigned nsym = symtab_sh.sh_size / sizeof(Elf32_Sym);
    for (unsigned i = 0; i < nsym; i++) {
        Elf32_Sym sym;
        memcpy(&sym, elf + symtab_sh.sh_offset + i * sizeof(Elf32_Sym), sizeof(sym));
        if (sym.st_name == 0) continue;
        if (strcmp(strtab + sym.st_name, "app_main") != 0) continue;
        if (sym.st_shndx >= eh.e_shnum || !img->sec[sym.st_shndx]) break;
        img->entry = (uint32_t)(uintptr_t)img->sec[sym.st_shndx] + sym.st_value;
        return 0;
    }

    app_unload(img);
    return fail("no app_main");
}

void app_unload(app_image_t *img)
{
    for (int i = 0; i < APP_MAX_SECTIONS; i++) {
        if (img->sec[i]) { vPortFree(img->sec[i]); img->sec[i] = NULL; }
    }
    img->entry = 0;
}
