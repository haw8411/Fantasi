#include "../cli.h"
#include "../../hal/hal.h"

#include <stdint.h>

static int cmd_memtest(int argc, char **argv)
{
    (void)argc; (void)argv;

    hal_test_region_t regions[HAL_TEST_REGIONS_MAX];
    int n = hal_test_regions(regions, HAL_TEST_REGIONS_MAX);
    if (n == 0) {
        cli_write("no testable regions on this platform\r\n");
        return 0;
    }

    uint32_t total_bytes = 0;
    int fail = 0;

    for (int r = 0; r < n; r++) {
        volatile uint32_t *base = (volatile uint32_t *)(uintptr_t)regions[r].addr;
        uint32_t words = regions[r].size / 4;
        total_bytes += words * 4;

        cli_printf("%-6s %6lu B @ 0x%08lX  ",
                   regions[r].name,
                   (unsigned long)(words * 4),
                   (unsigned long)regions[r].addr);

        uint32_t errors = 0;

        for (uint32_t i = 0; i < words; i++)
            base[i] = (uint32_t)(uintptr_t)&base[i];
        for (uint32_t i = 0; i < words; i++) {
            if (base[i] != (uint32_t)(uintptr_t)&base[i])
                errors++;
        }

        for (uint32_t i = 0; i < words; i++)
            base[i] = ~(uint32_t)(uintptr_t)&base[i];
        for (uint32_t i = 0; i < words; i++) {
            if (base[i] != ~(uint32_t)(uintptr_t)&base[i])
                errors++;
        }

        for (uint32_t i = 0; i < words; i++)
            base[i] = 0;

        if (errors) {
            cli_printf("FAIL (%lu errors)\r\n", (unsigned long)errors);
            fail = 1;
        } else {
            cli_write("OK\r\n");
        }
    }

    cli_printf("total: %lu B tested\r\n", (unsigned long)total_bytes);
    return fail;
}

CLI_COMMAND("memtest", "write/read-back test free SRAM", cmd_memtest);
