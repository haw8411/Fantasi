/* Fantasi app link script - relocatable merge.
 *
 * Apps are linked with `ld -r` (a relocatable link), NOT a final executable: the
 * output stays an ET_REL object whose relocations the firmware loader applies at
 * load time, placing the app wherever RAM is free. This script just merges the
 * per-function/-data input sections into tidy single .text/.rodata/.data/.bss
 * outputs so the loader allocates a handful of sections instead of dozens.
 *
 * Paired with these compile flags (see apps/Makefile):
 *   -mword-relocations  -> all relocations become R_ARM_ABS32 (no Thumb-branch
 *                          / MOVW / MOVT / GOT fixups for the loader to handle)
 *   -mlong-calls        -> calls go through a relocated literal, also ABS32
 *   -ffreestanding -nostdlib -fno-common -> no libc/startup pulled in; the only
 *                          imports are a few compiler helpers the loader resolves.
 */
SECTIONS {
    .text 0 : ALIGN(4) {
        *(.text) *(.text*)
        KEEP(*(.init)) KEEP(*(.fini))
    }
    .rodata : ALIGN(4) { *(.rodata) *(.rodata*) }
    .data   : ALIGN(4) { *(.data)   *(.data*)   }
    .bss    : ALIGN(4) { *(.bss)    *(.bss*)    *(COMMON) }

    /DISCARD/ : { *(.ARM.attributes) *(.comment) *(.ARM.exidx*) *(.note*) }
}
