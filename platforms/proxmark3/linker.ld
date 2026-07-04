/* Fantasi / Proxmark3 (AT91SAM7S)
 *
 * The PM3 bootloader owns 0x100000-0x102000 and must be preserved -
 * it is the only USB recovery path without a JTAG adapter. We link
 * our app starting at 0x102000, matching the stock `osimage` region.
 * Size is capped at 0x7E000 which is safe for both S256 (0x40000
 * total flash) and S512 (0x80000 total flash); the build will fail
 * at link time rather than silently corrupt the bootloader.
 *
 * RAM starts at 0x200000 but the first 64 bytes are reserved for the
 * ARM exception vector table we copy in at startup (see startup.S
 * and MC_RCR remap). The last 32 bytes of RAM are the PM3 bootloader
 * "common area" - untouched here so warm-boot handoff still works.
 */

PHDRS
{
    text PT_LOAD FLAGS(5);
    data PT_LOAD;
    bss  PT_LOAD;
}

ENTRY(Vector)

stack_size   = 0x1000;   /* 4 KB main/IRQ stack - FreeRTOS tasks have their own stacks */
vectors_size = 0x40;     /* ARM exception table + literal pool */
commonarea_size = 0x20;  /* PM3 bootloader communication region */

MEMORY {
    osimage    : ORIGIN = 0x00102000, LENGTH = 0x7E000
    ram_vec    : ORIGIN = 0x00200000, LENGTH = vectors_size
    ram        : ORIGIN = 0x00200000 + vectors_size,
                 LENGTH = 64K - vectors_size - commonarea_size
    commonarea : ORIGIN = 0x00200000 + 64K - commonarea_size, LENGTH = commonarea_size
}

_stack_end  = ORIGIN(ram) + LENGTH(ram);

_ram_start = ORIGIN(ram);
_ram_end   = ORIGIN(ram) + LENGTH(ram);

SECTIONS {
    .start : {
        KEEP(*(.startos))
    } >osimage :text

    .text : {
        *(.text .text.*)
        *(.glue_7) *(.glue_7t)
        *(.eh_frame)
        . = ALIGN(4);
    } >osimage :text

    .rodata : {
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } >osimage :text

    .data : {
        . = ALIGN(4);
        *(.data .data.*)
        *(.ramfunc)
        . = ALIGN(4);
    } >ram AT>osimage :data

    _eflash = LOADADDR(.data) + SIZEOF(.data);
    __data_src_start__ = LOADADDR(.data);
    __data_start__     = ADDR(.data);
    __data_end__       = __data_start__ + SIZEOF(.data);

    .bss : {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss .bss.*) *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
        end = .;                    /* newlib _sbrk anchor (unused; no malloc) */
        PROVIDE(_end = .);
    } >ram AT>ram :bss

    /* Stack grows down from _stack_end; heap grows up from __heap_start__. */
    ._user_heap (NOLOAD) : {
        . = ALIGN(8);
        __heap_start__ = .;
        . = ORIGIN(ram) + LENGTH(ram) - stack_size;
        __heap_end__ = .;
    } >ram :NONE

    .commonarea (NOLOAD) : {
        *(.commonarea)
    } >commonarea :NONE

    /DISCARD/ : { *(.ARM.attributes) *(.comment) *(.note.GNU-stack) }
}
