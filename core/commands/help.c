#include "../cli.h"

#include <string.h>

/* The command set is discovered from the cli_cmd linker section, whose order is
 * link order. Sort by name for a stable, readable listing. */
#define HELP_MAX 64

static int cmd_help(int argc, char **argv)
{
    (void)argc; (void)argv;

    const cli_command_t *list[HELP_MAX];
    size_t n = 0;
    for (const cli_command_t *c = __start_cli_cmd; c < __stop_cli_cmd && n < HELP_MAX; c++)
        list[n++] = c;

    for (size_t i = 0; i < n; i++)
        for (size_t j = i + 1; j < n; j++)
            if (strcmp(list[j]->name, list[i]->name) < 0) {
                const cli_command_t *t = list[i];
                list[i] = list[j];
                list[j] = t;
            }

    for (size_t i = 0; i < n; i++)
        cli_printf("  %-8s  %s\r\n", list[i]->name, list[i]->help);
    return 0;
}

CLI_COMMAND("help", "list commands", cmd_help);
