/* `ps` - list FreeRTOS tasks, Linux-style: id, name, state, priority, and the
 * stack headroom (minimum free stack ever seen), plus a heap summary. Needs
 * configUSE_TRACE_FACILITY. Per-task heap isn't tracked by FreeRTOS (the heap is
 * shared), so the per-task memory figure is stack headroom; `free` covers heap. */
#include "../cli.h"
#include "../../hal/hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

static char state_char(eTaskState s)
{
    switch (s) {
    case eRunning:   return 'R';   /* running        */
    case eReady:     return 'r';   /* ready          */
    case eBlocked:   return 'B';   /* blocked/waiting */
    case eSuspended: return 'S';   /* suspended      */
    case eDeleted:   return 'D';   /* awaiting cleanup */
    default:         return '?';
    }
}

static int cmd_ps(int argc, char **argv)
{
    (void)argc; (void)argv;
    cli_write("  PID  NAME          ST  PRI  STACKFREE\r\n");
    TaskStatus_t st;
    char name[configMAX_TASK_NAME_LEN];
    char line[80];
    UBaseType_t cursor = 0, n = 0;
    while (xTaskGetNextSystemStateEntry(&cursor, &st, name) == pdTRUE) {
        snprintf(line, sizeof(line), "  %3u  %-12s  %c   %2u  %6u B\r\n",
                 (unsigned)st.xTaskNumber, st.pcTaskName,
                 state_char(st.eCurrentState),
                 (unsigned)st.uxCurrentPriority,
                 (unsigned)((uint32_t)st.usStackHighWaterMark * sizeof(StackType_t)));
        cli_write(line);
        n++;
    }
    cli_printf("  %u tasks   heap: %u/%u B free (min-ever %u)\r\n",
               (unsigned)n, (unsigned)hal_free_heap_bytes(),
               (unsigned)configTOTAL_HEAP_SIZE,
               (unsigned)hal_min_ever_free_heap_bytes());
    return 0;
}

CLI_COMMAND("ps", "list tasks (state, priority, free stack) + heap summary", cmd_ps);
