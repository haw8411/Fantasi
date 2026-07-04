#include "../cli.h"

static int cmd_echo(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        cli_write(argv[i]);
        if (i + 1 < argc) cli_write(" ");
    }
    cli_write("\r\n");
    return 0;
}

CLI_COMMAND("echo", "print arguments back", cmd_echo);
