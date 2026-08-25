#include "../cli.h"
#include "../../hal/hal.h"

#include "FreeRTOS.h"
#include "task.h"

static int cmd_free(int argc, char **argv)
{
    (void)argc; (void)argv;
    /* Let the idle task reclaim terminated session workers before sampling. */
    vTaskDelay(pdMS_TO_TICKS(15));
    size_t now = hal_free_heap_bytes();
    size_t lo  = hal_min_ever_free_heap_bytes();
    cli_printf("heap: %u/%u B free (min-ever: %u)\r\n",
               (unsigned)now, (unsigned)configTOTAL_HEAP_SIZE, (unsigned)lo);
    HeapStats_t hs;
    vPortGetHeapStats(&hs);
    cli_printf("heap blocks: %u, largest: %u B\r\n",
               (unsigned)hs.xNumberOfFreeBlocks,
               (unsigned)hs.xSizeOfLargestFreeBlockInBytes);

    extern volatile uint32_t g_oom_count, g_oom_free_at_fail;
    if (g_oom_count)
        cli_printf("OOM: %u alloc failure(s), %u B free at last fail\r\n",
                   (unsigned)g_oom_count, (unsigned)g_oom_free_at_fail);

    hal_mem_region_t regions[HAL_MEM_REGIONS_MAX];
    int n = hal_mem_regions(regions, HAL_MEM_REGIONS_MAX);
    uint32_t app_free = 0;
    for (int i = 0; i < n; i++) {
        cli_printf("%-6s %6lu B total", regions[i].name,
                   (unsigned long)regions[i].total);
        if (regions[i].free)
            cli_printf(", %6lu B free", (unsigned long)regions[i].free);
        if (regions[i].note)
            cli_printf("  (%s)", regions[i].note);
        cli_write("\r\n");
        app_free += regions[i].free;
    }
    cli_printf("app: %lu B free\r\n", (unsigned long)app_free);
    return 0;
}

CLI_COMMAND("free", "report heap and SRAM free bytes", cmd_free);
