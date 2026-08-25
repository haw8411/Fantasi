/* Fantasi / Proxmark3 (AT91SAM7S)
 *
 * The PM3 bootloader owns 0x100000-0x102000 and must be preserved -
 * it is the only USB recovery path without a JTAG adapter. We link
 * our app at 0x102000. Flash map (S512 only, 0x100000-0x180000):
 *   0x102000-0x160000  app / osimage (376 KB, LENGTH 0x5E000 below)
 *   0x160000-0x180000  LittleFS storage (128 KB, see flash_storage.h)
 * The app is capped below the filesystem so a >256 KB image (with Berry)
 * can't overlap and corrupt it. An overflow is a link error.
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
    osimage    : ORIGIN = 0x00102000, LENGTH = 0x5E000   /* ends at 0x160000 = storage base */
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

    /* Keep USB descriptors in the first 256 KB flash plane for enumeration.
     * Berry code and tables may occupy the second plane. */
    .text : {
        *usb_descriptors.o(.rodata .rodata.* .text .text.*)
        . = ALIGN(4);
        *(EXCLUDE_FILE(*be_*.o) .text EXCLUDE_FILE(*be_*.o) .text.*)
        *(.glue_7) *(.glue_7t)
        *(.eh_frame)
        *(EXCLUDE_FILE(*be_*.o) .rodata EXCLUDE_FILE(*be_*.o) .rodata.*)
        . = ALIGN(4);
        __plane0_hot_end = .;
        /* EFC1 programming makes plane 1 unreadable. Keep non-Berry code and
         * constants in plane 0 so storage writes cannot stall system tasks. */
        *be_*.o(.text .text.* .rodata .rodata.*)
        . = ALIGN(4);
    } >osimage :text

    /* ARM exception unwind tables. Pulled in by picolibc's float printf (unwind-arm.c
     * references __exidx_start/__exidx_end); without these the link fails. */
    .ARM.extab : {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } >osimage :text

    .ARM.exidx : {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
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

    /* picolibc TLS template. Placed explicitly as NOLOAD (NOBITS) in RAM so it
     * never lands as an orphan LOAD section at the RAM VMA - which, lacking a
     * flash load address, makes objcopy -O binary pad the whole flash->RAM gap
     * into a ~1 MB .bin. The image is never copied at runtime (TLS is served
     * from fantasi_tls_block, see core/libc_glue.c), so dropping its init bytes
     * costs nothing; the sections still size the TLS segment for the ASSERT. */
    .tdata (NOLOAD) : {
        . = ALIGN(4);
        *(.tdata .tdata.* .gnu.linkonce.td.*)
    } >ram :bss

    .tbss (NOLOAD) : {
        *(.tbss .tbss.* .gnu.linkonce.tb.*) *(.tcommon)
    } >ram :bss

    .bss : {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss .bss.*) *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
        end = .;                    /* _sbrk anchor. The ._user_heap region below
                                     * is the FreeRTOS app heap: ucHeap is aliased
                                     * onto __heap_start__ and libc malloc is
                                     * wrapped onto it, so _sbrk stays unused. */
        PROVIDE(_end = .);
    } >ram AT>ram :bss

    /* Collect the picolibc TLS template (errno, rand state, localtime buffer)
     * into one explicitly-defined section so the ASSERT below can size it on any
     * toolchain: a bare SIZEOF(.tdata) errors out when a picolibc build emits no
     * .tdata. (INFO) keeps it out of the loadable image (no PT_LOAD / phdr); we
     * don't use the template at runtime anyway - TLS lives in core/libc_glue.c's
     * fantasi_tls_block via our own __aeabi_read_tp - this exists only to be
     * sized. */
    .tls_sizecheck (INFO) : {
        *(.tdata .tdata.*)
        *(.tbss .tbss.*)
    }

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

/* The picolibc TLS template must fit in core/libc_glue.c's fantasi_tls_block[128]
 * minus the 8-byte ARM EABI TCB. Fail the build if a new TLS consumer outgrows it
 * instead of overrunning the block at runtime. .tls_sizecheck is always defined
 * (above), so SIZEOF resolves even when the toolchain emits no .tdata/.tbss. */
ASSERT(SIZEOF(.tls_sizecheck) <= 120,
       "picolibc TLS exceeds fantasi_tls_block (core/libc_glue.c)")

/* Everything except Berry must stay in flash plane 0 (below 0x140000): plane 1
 * is unreadable while EFC1 programs LittleFS pages, and a stalled fetch there
 * freezes the core with interrupts masked. */
ASSERT(__plane0_hot_end <= 0x140000,
       "non-Berry code/rodata spilled into flash plane 1 (see linker.ld)")
