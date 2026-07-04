/* Fantasi / Proxmark3 (AT91SAM7S) startup
 *
 * The PM3 bootloader (0x100000-0x102000) has already set up the stack
 * and jumped here with ARM flash mirrored at 0x0. We then:
 *   1. Copy an 8-entry ARM exception vector table from flash to the
 *      start of SRAM (0x200000).
 *   2. Write MC_RCR = 1 to remap SRAM (not flash) at 0x0, so the ARM
 *      core now fetches exception vectors from our table.
 *   3. Copy .data from flash to RAM and clear .bss.
 *   4. Branch to main().
 *
 * Without step 1+2 the SWI vector still points at the bootloader's
 * `b software_interrupt` infinite loop, which breaks FreeRTOS's
 * taskYIELD() path on ARM7.
 *
 * The IRQ vector uses the standard AT91 trick: `ldr pc, [pc, #-0xF20]`
 * reads AIC_IVR (0xFFFFF100) and jumps to the currently-vectored
 * AIC handler. FreeRTOS + our USB IRQ register into the AIC.
 */

    .syntax unified

    .section .startos, "ax"
    .global Vector
    .global __vectors_src

    /* The PM3 bootrom branches to _osimage_entry with bit 0 forced
     * to 1 (see bootrom.c: `bx (_osimage_entry | 0x1)`), which means
     * we MUST enter in Thumb mode. Put a tiny Thumb trampoline at
     * offset 0 of the osimage and jump to the ARM-mode startup. */
    .thumb
    .thumb_func
Vector:
    ldr     r0, =_arm_start
    bx      r0                      @ bit 0 of r0 is clear → ARM mode

    .align 2
    .arm
_arm_start:
    /* Initialise one stack per ARM processor mode. The bootrom sets
     * only the supervisor SP; every other mode's banked SP is still at
     * its power-on-random value (we've seen sp_irq = 0xfffffffb here).
     * The FIRST IRQ (FreeRTOS's 1 ms PIT tick) pushes onto IRQ-banked
     * SP → writes to unmapped memory → data abort → spin in
     * __fantasi_unhandled. USB enumeration starts briefly before the
     * first tick fires, which is why we saw "enumerate OK then die".
     *
     * Carve the top of the ram region into four sub-stacks. FIQ /
     * Abort / Undefined are only needed for fault safety but cost 64
     * bytes each and prevent a second crash when a fault fires. */
    msr     cpsr_c, #0xD1                   @ FIQ mode,    IRQ+FIQ masked
    ldr     sp, =_stack_end

    msr     cpsr_c, #0xD2                   @ IRQ mode
    ldr     sp, =_stack_end
    sub     sp, sp, #0x40                   @ reserve 0x40 above for FIQ

    msr     cpsr_c, #0xD7                   @ Abort mode
    ldr     sp, =_stack_end
    sub     sp, sp, #0x140                  @ FIQ + IRQ room above (0x40 + 0x100)

    msr     cpsr_c, #0xDB                   @ Undefined mode
    ldr     sp, =_stack_end
    sub     sp, sp, #0x180                  @ + 0x40 abort room

    msr     cpsr_c, #0xD3                   @ back to Supervisor
    ldr     sp, =_stack_end
    sub     sp, sp, #0x1C0                  @ everything above is per-mode

    /* Copy ARM exception table (32 bytes) + literal pool (28 bytes +
     * 4 pad = 32) = 64 bytes from flash to 0x200000. */
    ldr     r0, =__vectors_src
    mov     r1, #0x200000
    mov     r2, #16                 @ 16 words = 64 bytes
1:  ldr     r3, [r0], #4
    str     r3, [r1], #4
    subs    r2, r2, #1
    bne     1b

    /* Swap: SRAM now mirrored at 0x0 (was flash).  Single write to
     * MC_RCR at 0xFFFFFF00. After this, exception fetches hit our
     * table in SRAM. */
    ldr     r0, =0xFFFFFF00
    mov     r1, #1
    str     r1, [r0]

    /* Copy .data from LMA (flash) to VMA (RAM). Symbols come from the
     * linker script. */
    ldr     r0, =__data_src_start__
    ldr     r1, =__data_start__
    ldr     r2, =__data_end__
2:  cmp     r1, r2
    bhs     3f
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    b       2b

3:  /* Zero .bss. */
    ldr     r0, =__bss_start__
    ldr     r1, =__bss_end__
    mov     r2, #0
4:  cmp     r0, r1
    bhs     5f
    str     r2, [r0], #4
    b       4b

5:  /* Hand off to C. main() doesn't return; hang if it does. */
    bl      main
6:  b       6b


/* ARM exception table source: loaded from flash, copied to SRAM at 0x0.
 * Order is fixed by ARMv4 architecture: reset, undef, swi, pabt, dabt,
 * reserved, irq, fiq. Each `ldr pc, <label>` becomes a PC-relative
 * literal load in the assembled code. */
    .align 2
__vectors_src:
    ldr     pc, _vec_reset
    ldr     pc, _vec_undef
    ldr     pc, _vec_swi
    ldr     pc, _vec_pabt
    ldr     pc, _vec_dabt
    nop                             @ reserved slot
    ldr     pc, [pc, #-0xF20]       @ IRQ → AIC_IVR (0xFFFFF100)
    ldr     pc, _vec_fiq

_vec_reset: .word  Vector
_vec_undef: .word  __fantasi_unhandled
_vec_swi:   .word  vPortYieldProcessor
_vec_pabt:  .word  __fantasi_unhandled
_vec_dabt:  .word  __fantasi_unhandled
_vec_fiq:   .word  __fantasi_unhandled
            .word  0               @ pad to 64 bytes


    .section .text
    .global __fantasi_unhandled
    .type __fantasi_unhandled, %function
__fantasi_unhandled:
    b       __fantasi_unhandled
