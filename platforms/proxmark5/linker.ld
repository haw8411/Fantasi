/* Fantasi / Proxmark5 (AT32F435, Cortex-M4)
 *
 * Flash starts at 0x08000000 (1024 KB on the xG variant the PM5 ships).
 * The AT32 ROM system bootloader lives in system memory at 0x1FFF0000,
 * not in user flash, so we link the application at the flash base and
 * still keep the ROM DFU as the recovery path (reached via the .noinit
 * magic + reset in startup.c, or physically via the BOOT pins).
 *
 * The 1024 KB flash is dual-bank: bank1 0x08000000..0x08080000 and bank2
 * 0x08080000..0x08100000, 2 KB sectors (RM Table 5-2). We link the app into
 * bank1 only and keep the whole LittleFS storage region in bank2 (see
 * flash_storage.h). Two wins from that split: an image that outgrows bank1
 * is a link error by construction, and the runtime flash driver only ever
 * erases/programs bank2 - never the bank the code was linked into.
 *
 * SRAM: a User System Data option byte splits a shared budget between zero-wait
 * flash and SRAM. The factory default is 384 KB SRAM (with the largest zero-wait
 * flash region); trading up to 512 KB SRAM costs zero-wait flash. We link the
 * 384 KB default - ample here, and it keeps the whole image in zero-wait flash.
 */

ENTRY(Reset_Handler)

_stack_size = 0x1000;   /* 4 KB main/IRQ stack */

MEMORY {
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K       /* bank1; storage lives in bank2 */
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 384K       /* SRAM default floor */
}

_stack_end = ORIGIN(RAM) + LENGTH(RAM);   /* top-of-stack for vector[0] */

_ram_start = ORIGIN(RAM);
_ram_end   = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS {
    .isr_vector : {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text : {
        . = ALIGN(4);
        *(.text .text.*)
        *(.glue_7) *(.glue_7t)
        KEEP(*(.init)) KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata : {
        . = ALIGN(4);
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array : {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH
    .init_array : {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH
    .fini_array : {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH

    _sidata = LOADADDR(.data);

    .data : {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*)
        *(.ramfunc .ramfunc.*)      /* RAM-resident hot code (RFID decoders) */
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    _eflash = LOADADDR(.data) + SIZEOF(.data);

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = .;
        *(.bss .bss.*) *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = .;
        end = .;                    /* _sbrk anchor. The ._user_heap_stack region
                                     * below is the FreeRTOS app heap: ucHeap is
                                     * aliased onto __heap_start__ and libc malloc
                                     * is wrapped onto it, so _sbrk stays unused. */
        PROVIDE(_end = .);
    } >RAM

    /* Collect the picolibc TLS template (errno, rand state, localtime buffer)
     * into one explicitly-defined section so the ASSERT below can size it on any
     * toolchain: a bare SIZEOF(.tdata) errors out when a picolibc build emits no
     * .tdata. (INFO) keeps it out of the loadable image; placed here (RAM still
     * has room) it doesn't overlay a real section. We don't use the template at
     * runtime anyway - TLS lives in core/libc_glue.c's fantasi_tls_block via our
     * own __aeabi_read_tp - this exists only to be sized. */
    .tls_sizecheck (INFO) : {
        *(.tdata .tdata.*)
        *(.tbss .tbss.*)
    }

    /* .noinit: SRAM that survives a warm reset and is not zeroed by
     * Reset_Handler. Holds the DFU handoff magic - writing it from a
     * running firmware and reading it again from early reset is how we
     * enter the AT32 system-memory bootloader (0x1FFF0000) on the next
     * reset. */
    .noinit (NOLOAD) : {
        . = ALIGN(4);
        *(.noinit .noinit.*)
        . = ALIGN(4);
    } >RAM

    ._user_heap_stack (NOLOAD) : {
        . = ALIGN(8);
        __heap_start__ = .;
        . = ORIGIN(RAM) + LENGTH(RAM) - _stack_size;
        __heap_end__ = .;
        . = . + _stack_size;
        . = ALIGN(8);
    } >RAM

    /DISCARD/ : { *(.ARM.attributes) *(.comment) *(.note.GNU-stack) }
}

/* The picolibc TLS template must fit in core/libc_glue.c's fantasi_tls_block[128]
 * minus the 8-byte ARM EABI TCB. Fail the build if a new TLS consumer outgrows it
 * instead of overrunning the block at runtime. .tls_sizecheck is always defined
 * (above), so SIZEOF resolves even when the toolchain emits no .tdata/.tbss. */
ASSERT(SIZEOF(.tls_sizecheck) <= 120,
       "picolibc TLS exceeds fantasi_tls_block (core/libc_glue.c)")
