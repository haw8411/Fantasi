#include "../cli.h"
#include "../../hal/storage/hal_storage.h"

#include <string.h>

/* Erase all internal LittleFS storage. Destructive, so it will not fire on a bare
 * `format` - the caller must confirm with the explicit target `internal`. */
static int cmd_format(int argc, char **argv)
{
    if (argc < 2 || strcmp(argv[1], "internal") != 0) {
        cli_write("format: Erases all internal storage.\r\n"
                  "syntax: format internal\r\n");
        return 1;
    }

    cli_write("formatting internal storage...\r\n");
    cli_flush();
    int err = hal_storage_format();
    if (err) cli_printf("format failed (%d)\r\n", err);
    else     cli_write("internal storage erased\r\n");
    return err ? 1 : 0;
}

CLI_COMMAND("format", "erase internal storage (format internal)", cmd_format);
