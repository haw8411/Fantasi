#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION            1
/* No idle-hook WFE: on nRF52 the CPU clock (and SysTick with it) stops during
 * WFE, so an un-accounted idle-hook sleep silently freezes the tick. All
 * idle sleeping goes through the tickless hook,
 * which measures elapsed time on RTC1 and credits it with vTaskStepTick. */
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0

/* Tickless idle, custom implementation (platforms/chameleon/power.c): RTC1 on
 * the LFXO as the wake timer, SysTick suppressed, SoftDevice-safe sleep. */
#define configUSE_TICKLESS_IDLE         2
#ifndef __ASSEMBLER__
extern void cu_suppress_ticks_and_sleep(uint32_t expected_idle_ticks);
#endif
#define portSUPPRESS_TICKS_AND_SLEEP(x) cu_suppress_ticks_and_sleep(x)
#define configCPU_CLOCK_HZ              ((unsigned long)64000000)
#define configTICK_RATE_HZ              ((TickType_t)1000)
#define configMAX_PRIORITIES            5
/* 256: idle needs it for the tickless sleep path + FPU exception frame. cli/usb/
 * proto pin their stacks in the Makefile; only idle, pwrbtn, and timer still size
 * off this. */
#define configMINIMAL_STACK_SIZE        ((unsigned short)256)
/* Elastic app heap. ucHeap (heap_4.c) is aliased onto the linker heap region,
 * and this size is its runtime span - end of .bss to the stack - so the FreeRTOS
 * heap uses all app RAM above the SoftDevice with nothing stranded. libc malloc
 * (float printf's conversion buffers) is wrapped onto this same heap
 * (core/libc_glue.c + the --wrap flags in the Makefile), so there is no separate
 * arena. The heap grows at the top of RAM, away from the SoftDevice's reservation
 * at the bottom. */
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
#define INCLUDE_xTaskGetCurrentTaskHandle 1

/* nRF52840 has 3 priority bits (8 priority levels, not 16 like STM32). */
#define configPRIO_BITS                         3
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         7
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    3
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define vPortSVCHandler      SVC_Handler
#define xPortPendSVHandler   PendSV_Handler
#define xPortSysTickHandler  SysTick_Handler

#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

#endif
