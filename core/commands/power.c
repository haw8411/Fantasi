/* power - sleep governance status + control.
 *
 *   power                  show allowed depth, inhibitor votes, sleep stats
 *   power sleep on|off     enable/disable CPU sleep (persisted, sleep=0|1)
 *   power off-timeout <s>  idle auto-power-off in seconds, 0 = never
 *                          (persisted, offtimeout=<s>; battery platforms only)
 */

#include "../cli.h"
#include "../app_run.h"
#include "../../hal/hal_power.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>
#include <string.h>

static const char *depth_name(hal_sleep_depth_t d)
{
    switch (d) {
    case HAL_SLEEP_NONE:  return "none";
    case HAL_SLEEP_LIGHT: return "light";
    case HAL_SLEEP_DEEP:  return "deep";
    }
    return "?";
}

static const char *const wake_name[PWR_WAKE_COUNT] = {
    "timer", "usb", "radio", "button", "other",
};

static int cmd_power(int argc, char **argv)
{
    if (argc >= 3 && strcmp(argv[1], "sleep") == 0) {
        bool on = strcmp(argv[2], "on") == 0;
        if (!on && strcmp(argv[2], "off") != 0) {
            cli_printf("usage: power sleep on|off\r\n");
            return -1;
        }
        if (pwr_set_sleep_enabled(on) != 0)
            cli_printf("warning: not persisted (storage unavailable)\r\n");
        cli_printf("sleep %s\r\n", on ? "on" : "off");
        return 0;
    }

    if (argc >= 3 && strcmp(argv[1], "off-timeout") == 0) {
        uint32_t s = (uint32_t)strtoul(argv[2], NULL, 10);
        if (pwr_set_off_timeout_s(s) != 0)
            cli_printf("warning: not persisted (storage unavailable)\r\n");
        if (s) cli_printf("off-timeout %lus\r\n", (unsigned long)s);
        else   cli_printf("off-timeout never\r\n");
        return 0;
    }

    if (argc >= 2) {
        cli_printf("usage: power [sleep on|off] [off-timeout <s>]\r\n");
        return -1;
    }

    pwr_stats_t st;
    pwr_get_stats(&st);
    TickType_t now = xTaskGetTickCount();

    cli_printf("sleep:        %s\r\n", pwr_sleep_enabled() ? "on" : "off");
    cli_printf("allowed:      %s\r\n", depth_name(pwr_allowed_depth()));
    if (pwr_off_timeout_s())
        cli_printf("off-timeout:  %lus\r\n", (unsigned long)pwr_off_timeout_s());
    else
        cli_printf("off-timeout:  never\r\n");

    cli_printf("inhibitors:  ");
    bool any = false;
    for (int c = 0; c < PWR_CLIENT_COUNT; c++) {
        int v = pwr_client_votes((pwr_client_t)c);
        if (v > 0) {
            cli_printf(" %s=%d", pwr_client_name((pwr_client_t)c), v);
            any = true;
        }
    }
    if (app_running_pid() >= 0) { cli_printf(" app"); any = true; }
    cli_printf(any ? "\r\n" : " none\r\n");

    /* slept_ticks (platform-measured actual sleep) over uptime ticks. Clamped:
     * sub-tick measurement granularity can nudge the ratio a hair over 100%,
     * which is nonsensical to display. */
    uint32_t pct10 = 0;
    if (now > 0) {
        pct10 = (uint32_t)((uint64_t)st.slept_ticks * 1000 / now);
        if (pct10 > 1000) pct10 = 1000;
    }
    cli_printf("sleeps:       light=%lu deep=%lu\r\n",
               (unsigned long)st.light_entries, (unsigned long)st.deep_entries);
    cli_printf("time asleep:  %lu/%lu ticks (%lu.%lu%%)\r\n",
               (unsigned long)st.slept_ticks, (unsigned long)now,
               (unsigned long)(pct10 / 10), (unsigned long)(pct10 % 10));
    cli_printf("wakes:       ");
    for (int w = 0; w < PWR_WAKE_COUNT; w++)
        cli_printf(" %s=%lu", wake_name[w], (unsigned long)st.wake[w]);
    cli_printf("\r\n");
    return 0;
}

CLI_COMMAND("power", "sleep status/control (power sleep on|off)", cmd_power);
