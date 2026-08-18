#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION            1
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0
#define configCPU_CLOCK_HZ              ((unsigned long)32000000)  /* HSE 32 MHz; see platforms/flipper/system.c */
#define configTICK_RATE_HZ              ((TickType_t)1000)
#define configMAX_PRIORITIES            5
/* 256 words: the idle task runs the kernel tickless path on this stack, and
 * the backlight task comes within ~48 B of overflow at the 128-word default. */
#define configMINIMAL_STACK_SIZE        ((unsigned short)256)

/* Tickless idle. The port's weak vPortSuppressTicksAndSleep is overridden in
 * platforms/flipper/power.c: light path = the kernel's own SysTick algorithm
 * (plain WFI, CPU2-safe, SysTick keeps counting); deep path = Stop 2 with the
 * LPTIM1 wake timer and the stock CPU1/CPU2 HSEM protocol, gated to battery
 * operation only. */
#define configUSE_TICKLESS_IDLE         1
/* Elastic app heap. ucHeap (heap_4.c) is aliased onto the linker heap region,
 * and this size is its runtime span - end of .bss to the stack - so the FreeRTOS
 * heap uses all free SRAM1 with nothing stranded. libc malloc (float printf's
 * conversion buffers) is wrapped onto this same heap (core/libc_glue.c + the
 * --wrap flags in the Makefile), so there is no separate arena. */
#ifndef __ASSEMBLER__
extern unsigned char __heap_start__, __heap_end__;   /* linker heap region bounds */
#endif
#define configAPPLICATION_ALLOCATED_HEAP  1
#define configTOTAL_HEAP_SIZE           ((size_t)(&__heap_end__ - &__heap_start__))
#define configMAX_TASK_NAME_LEN         12
#define configUSE_TRACE_FACILITY        1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
/* App task stack (words). 4 KB overflows on the RFID MFC-emu auth path (recv Miller DSP + crypto1 + streamed
 * reply build). The Flipper has ample RAM; give it 8 KB. */
#define APP_TASK_STACK_WORDS            2048
/* Fire the RFID emu ISR-shutdown the instant its task is deleted (abrupt kill / USB disconnect never runs
 * emu_teardown, leaving the EXTI4 ISR notifying a freed task handle -> heap corruption -> USB stall). */
extern void rfid_emu_task_deleted(void *tcb);
#define traceTASK_DELETE(pxTCB)         rfid_emu_task_deleted((void *)(pxTCB))
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

#define configUSE_TIMERS                1
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
#define INCLUDE_xTimerPendFunctionCall  1
#define INCLUDE_xQueueGetMutexHolder    1

/* Cortex-M specific: 4 bits of priority grouping, priorities 0..15.
 * Keep kernel at lowest possible priority (15) and SVC at highest (0). */
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Map FreeRTOS Cortex-M port handlers to the CMSIS handler names used
 * in our vector table. */
#define vPortSVCHandler      SVC_Handler
#define xPortPendSVHandler   PendSV_Handler
#define xPortSysTickHandler  SysTick_Handler

/* On assert failure, reset the device (via the same path the stack-overflow and
 * malloc-failed hooks use) so it self-recovers instead of spinning forever with
 * dead USB. */
void fantasi_reset(void);
#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); fantasi_reset(); }

#endif
