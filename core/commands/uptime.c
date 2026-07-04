#include "../cli.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

static int cmd_uptime(int argc, char **argv)
{
    (void)argc; (void)argv;
    TickType_t ticks = xTaskGetTickCount();
    uint32_t total_s = ticks / configTICK_RATE_HZ;
    uint32_t ms = (ticks % configTICK_RATE_HZ) * 1000 / configTICK_RATE_HZ;
    uint32_t d = total_s / 86400;
    uint32_t h = (total_s % 86400) / 3600;
    uint32_t m = (total_s % 3600) / 60;
    uint32_t s = total_s % 60;
    if (d > 0)
        cli_printf("%lud %luh %lum %lu.%03lus\r\n",
                   (unsigned long)d, (unsigned long)h,
                   (unsigned long)m, (unsigned long)s, (unsigned long)ms);
    else if (h > 0)
        cli_printf("%luh %lum %lu.%03lus\r\n",
                   (unsigned long)h, (unsigned long)m,
                   (unsigned long)s, (unsigned long)ms);
    else if (m > 0)
        cli_printf("%lum %lu.%03lus\r\n",
                   (unsigned long)m, (unsigned long)s, (unsigned long)ms);
    else
        cli_printf("%lu.%03lus\r\n", (unsigned long)s, (unsigned long)ms);
    return 0;
}

CLI_COMMAND("uptime", "show time since boot", cmd_uptime);
