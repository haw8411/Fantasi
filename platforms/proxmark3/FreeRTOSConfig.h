#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* AT91SAM7S / PM3 runs at 48 MHz from the 18.432 MHz crystal via PLL
 * (18.432 * 73 / 14 / 2 = 48.054 MHz - close enough). The bootloader
 * has already configured the clock by the time we reach Vector(). */
#define configCPU_CLOCK_HZ              ((unsigned long)48054857)
#define configTICK_RATE_HZ              ((TickType_t)1000)

#define configUSE_PREEMPTION            1
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0   /* WDT is kicked in the launcher + sniff loops, not the tick
                                             * ISR - a per-tick hook preempts the 212 kB/s sniff capture
                                             * loop and cuts its modulation sensitivity (~900->~200) */
#define configMAX_PRIORITIES            5
/* ARM7 port saves a 72-byte context frame on the task's stack on each
 * IRQ (tick) and each SWI (taskYIELD). With configIDLE_SHOULD_YIELD=1
 * the IDLE task yields once per iteration, so even at 128 words (512
 * B, the Cortex-M default) stack pressure is high. 256 words (1 KB)
 * matches the upstream AT91SAM7S demo and leaves headroom for nested
 * tick-during-SWI. */
#define configMINIMAL_STACK_SIZE        ((unsigned short)256)
/* Elastic app heap. ucHeap (heap_4.c) is aliased onto the linker ._user_heap
 * region, and this size is its runtime span - end of .bss to the stack - so the
 * FreeRTOS heap uses every free byte of the 64 KB SRAM with nothing stranded.
 * libc malloc - including float printf's conversion buffers - is wrapped onto
 * this same heap (core/libc_glue.c + the --wrap flags in the Makefile), so there
 * is no separate arena. */
#ifndef __ASSEMBLER__
extern unsigned char __heap_start__, __heap_end__;   /* linker heap region bounds */
#endif
#define configAPPLICATION_ALLOCATED_HEAP  1
#define configTOTAL_HEAP_SIZE           ((size_t)(&__heap_end__ - &__heap_start__))
#define configMAX_TASK_NAME_LEN         12
#define configUSE_TRACE_FACILITY        1
#define configUSE_16_BIT_TICKS          0
#define configIDLE_SHOULD_YIELD         1
#define configUSE_MUTEXES               1
#define configUSE_RECURSIVE_MUTEXES     1
#define configUSE_COUNTING_SEMAPHORES   1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 1
#define configQUEUE_REGISTRY_SIZE       8
#define configCHECK_FOR_STACK_OVERFLOW  2
#define configUSE_MALLOC_FAILED_HOOK    1
#define configUSE_NEWLIB_REENTRANT      0
#define configSUPPORT_STATIC_ALLOCATION 0

/* Software timers off: nothing on the PM3 creates one (only the Flipper's BLE
 * adv_timer uses xTimer*), so leaving them on would spawn the timer daemon task
 * to service nothing - ~2 KB of heap wasted out of the 64 KB SRAM. Set back to 1 if
 * a future PM3 feature needs a timer or xTimerPendFunctionCall. */
#define configUSE_TIMERS                0
#define configTIMER_TASK_PRIORITY       3
#define configTIMER_QUEUE_LENGTH        10
#define configTIMER_TASK_STACK_DEPTH    (configMINIMAL_STACK_SIZE * 2)

#define INCLUDE_vTaskPrioritySet        1
#define INCLUDE_uxTaskPriorityGet       1
#define INCLUDE_vTaskDelete             1
#define INCLUDE_vTaskSuspend            1
#define INCLUDE_vTaskDelayUntil         1
#define INCLUDE_vTaskDelay              1
#define INCLUDE_xTaskGetSchedulerState  1
#define INCLUDE_xTimerPendFunctionCall  0   /* needs the timer daemon; unused on PM3 (configUSE_TIMERS 0) */

/* ARM7 port: tick comes from PIT via AIC. Nothing here about Cortex-M
 * priority bits - AT91's AIC has 8 priority levels, handled by the
 * port internally. */
#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

/* THUMB_INTERWORK is deliberately NOT defined.
 *
 * We compile ALL our C with -mcpu=arm7tdmi -mthumb-interwork (no
 * -mthumb), so every function is ARM-mode. The GCC flag is about
 * generating interwork-safe BL/BX veneers - it doesn't actually
 * produce Thumb code unless -mthumb is also passed. Task entry
 * points (cli_task, platform_usb_task, prvIdleTask) are therefore
 * all ARM, with their addresses even-aligned and bit 0 clear.
 *
 * pxPortInitialiseStack's THUMB_INTERWORK branch ORs portTHUMB_MODE_BIT
 * (0x20) into SPSR, so tasks begin with CPSR.T=1. portRESTORE_CONTEXT
 * returns via `subs pc, lr, #4` which copies SPSR→CPSR - so on the
 * very first task resume the CPU switches to Thumb and tries to
 * decode ARM opcodes as Thumb, faulting with an Undefined Instruction.
 *
 * With THUMB_INTERWORK undefined, SPSR stays at 0x1F (System, ARM)
 * and tasks enter in ARM mode, matching our build. portmacro.h's
 * fallback inline-asm portDISABLE_INTERRUPTS / portENABLE_INTERRUPTS
 * are fine because every caller site is ARM. */

#endif
