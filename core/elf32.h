/* Just enough ELF32 to load a little-endian ARM relocatable object. */
#ifndef CORE_ELF32_H
#define CORE_ELF32_H

#include <stdint.h>

typedef uint32_t Elf32_Addr;
typedef uint32_t Elf32_Word;
typedef int32_t  Elf32_Sword;
typedef uint16_t Elf32_Half;
typedef uint32_t Elf32_Off;

typedef struct {
    unsigned char e_ident[16];
    Elf32_Half    e_type;
    Elf32_Half    e_machine;
    Elf32_Word    e_version;
    Elf32_Addr    e_entry;
    Elf32_Off     e_phoff;
    Elf32_Off     e_shoff;
    Elf32_Word    e_flags;
    Elf32_Half    e_ehsize;
    Elf32_Half    e_phentsize;
    Elf32_Half    e_phnum;
    Elf32_Half    e_shentsize;
    Elf32_Half    e_shnum;
    Elf32_Half    e_shstrndx;
} Elf32_Ehdr;

typedef struct {
    Elf32_Word sh_name;
    Elf32_Word sh_type;
    Elf32_Word sh_flags;
    Elf32_Addr sh_addr;
    Elf32_Off  sh_offset;
    Elf32_Word sh_size;
    Elf32_Word sh_link;
    Elf32_Word sh_info;
    Elf32_Word sh_addralign;
    Elf32_Word sh_entsize;
} Elf32_Shdr;

typedef struct {
    Elf32_Word    st_name;
    Elf32_Addr    st_value;
    Elf32_Word    st_size;
    unsigned char st_info;
    unsigned char st_other;
    Elf32_Half    st_shndx;
} Elf32_Sym;

typedef struct {
    Elf32_Addr r_offset;
    Elf32_Word r_info;
} Elf32_Rel;

#define ELF_MAG0 0x7f
#define ELFCLASS32   1
#define ELFDATA2LSB  1
#define ET_REL       1
#define EM_ARM       40

#define SHT_PROGBITS 1
#define SHT_SYMTAB   2
#define SHT_STRTAB   3
#define SHT_NOBITS   8
#define SHT_REL      9

#define SHF_ALLOC     0x2

#define SHN_UNDEF 0
#define SHN_ABS   0xfff1

#define ELF32_R_SYM(i)   ((i) >> 8)
#define ELF32_R_TYPE(i)  ((i) & 0xff)

#define R_ARM_ABS32   2
#define R_ARM_TARGET1 38   /* same fixup as ABS32 */
#define R_ARM_V4BX    40   /* ARMv4 BX marker; no-op on ARMv4T (AT91SAM7S) */

#endif /* CORE_ELF32_H */
